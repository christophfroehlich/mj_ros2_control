////////////////////////////////////////////////////////////////////////////////
// Copyright 2022 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    mujoco_simulator.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2022/02/01
 *
 */
//-----------------------------------------------------------------------------

#include "mj_ros2_control/mujoco_simulator.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <iostream>
#include <thread>

namespace mj_ros2_control
{
std::atomic<bool> first_write{false};

MuJoCoSimulator::MuJoCoSimulator() {}

void MuJoCoSimulator::controlCB(const mjModel * m, mjData * d)
{
  getInstance().controlCBImpl(m, d);
}

void MuJoCoSimulator::controlCBImpl([[maybe_unused]] const mjModel * m, mjData * d)
{
  command_mutex.lock();

  for (size_t i = 0; i < joint_control_methods_.size(); ++i) {

    if (joint_control_methods_[i] & POSITION) {
      if (std::isfinite(pos_cmd[i])) {
        d->ctrl[i] = pos_cmd[i];
      } else {
        std::cout << "commands not finite" << std::endl;
        d->ctrl[i] = 0.;
      }
    }

    if (joint_control_methods_[i] & VELOCITY) {
      if (std::isfinite(vel_cmd[i])) {
        d->ctrl[i] = vel_cmd[i];
      } else {
        std::cout << "commands not finite" << std::endl;
        d->ctrl[i] = 0.;
      }
    }

    if (joint_control_methods_[i] & EFFORT) {
      if (std::isfinite(eff_cmd[i])) {
        d->ctrl[i] = eff_cmd[i];
      } else {
        std::cout << "commands not finite" << std::endl;
        d->ctrl[i] = 0.;
      }
    }

    if (joint_control_methods_[i] & POS_VELOCITY) {
      // Joint-level impedance control
      if (std::isfinite(pos_cmd[i]) && std::isfinite(vel_cmd[i])) {
        d->ctrl[i] = stiff[i] * (pos_cmd[i] - d->qpos[i]) +       // stiffness
          damp[i] * (vel_cmd[i] - d->qvel[i]);       // damping
      } else {
        std::cout << "commands/params not finite" << std::endl;
        d->ctrl[i] = -damp[i] * d->actuator_velocity[i];        // damping
      }
      if (!std::isfinite(d->ctrl[i])) {
        d->ctrl[i] = 0.0;
        std::cout << "output not finite" << std::endl;
      }
    }
  }
  command_mutex.unlock();
}

int MuJoCoSimulator::simulate(
  const std::string & model_xml,
  const hardware_interface::HardwareInfo & hw_info)
{
  return getInstance().simulateImpl(model_xml, hw_info);
}

int MuJoCoSimulator::simulateImpl(
  const std::string & model_xml,
  const hardware_interface::HardwareInfo & hw_info)
{
  // Make sure that the ros2_control system_interface only gets valid data in read().
  // We lock until we are done with simulation setup.
  state_mutex.lock();

  // parse XML
  char error[1000] = "";
  // mjSpec * spec = mj_parseXML(model_xml.c_str(), nullptr, error, 1000);
  mjSpec * spec = mj_parseXMLString(hw_info.original_xml.c_str(), nullptr, error, 1000);

  // add actuator settings to model
  auto def = mjs_getSpecDefault(spec);
  def->actuator->biastype = mjBIAS_AFFINE;
  def->actuator->gaintype = mjGAIN_FIXED;
  def->actuator->trntype = mjTRN_JOINT;
  double kp, kv;
  std::vector<double> init_val;
  joint_control_methods_.resize(hw_info.joints.size());
  stiff.resize(hw_info.joints.size());
  damp.resize(hw_info.joints.size());

  for (size_t j = 0; j < hw_info.joints.size(); ++j) {
    auto joint_info = hw_info.joints[j];

    const std::string joint_name = joint_info.name;
    auto get_initial_value =
      [joint_name](const hardware_interface::InterfaceInfo & interface_info) {
        double initial_value{0.0};
        if (!interface_info.initial_value.empty()) {
          try {
            initial_value = std::stod(interface_info.initial_value);
            std::cout << "found initial value: " << initial_value << std::endl;
          } catch (std::invalid_argument &) {
            std::cout <<
              "Failed converting initial_value string to real number for the joint "
                      << joint_name
                      << " and state interface " << interface_info.name
                      << ". Actual value of parameter: " << interface_info.initial_value
                      << ". Initial value will be set to 0.0" << std::endl;
          }
        }
        return initial_value;
      };

    // parse initial values
    for (unsigned int i = 0; i < joint_info.state_interfaces.size(); ++i) {
      if (joint_info.state_interfaces[i].name == "position") {
        double joint_init_val = get_initial_value(joint_info.state_interfaces.at(0));
        if (std::isfinite(joint_init_val)) {
          init_val.push_back(joint_init_val);
        } else {
          init_val.push_back(0.0);
        }
      }
    }

    // TODO(anyone): rewrite logic of size/type checks
    if (joint_info.command_interfaces.size() > 2) {
      mju_error("Command interface combination is not supported");
      return 1;
    }
    if (
      (joint_info.command_interfaces.size() == 1 && (
        joint_info.command_interfaces.at(0).name == "position" ||
        joint_info.command_interfaces.at(0).name == "velocity" ||
        joint_info.command_interfaces.at(0).name == "effort")) ||
      (joint_info.command_interfaces.size() == 2 &&
      joint_info.command_interfaces.at(0).name == "position" &&
      joint_info.command_interfaces.at(1).name == "velocity"))
    {
      mjsActuator * pact = mjs_addActuator(spec, def);
      mjs_setString(pact->target, joint_name.c_str());
      if (joint_info.command_interfaces.size() == 1) {
        if (joint_info.command_interfaces.at(0).name == "position") {
          mjs_setString(pact->name, (joint_name + "_pos").c_str());
          joint_control_methods_[j] |= POSITION;
          // https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-position
          auto it = joint_info.parameters.find("p");
          if (it != joint_info.parameters.end()) {
            kp = std::stod(it->second);
          } else {
            kp = 1.;
          }
          it = joint_info.parameters.find("d");
          if (it != joint_info.parameters.end()) {
            kv = std::stod(it->second);
          } else {
            kv = 0.;
          }
          pact->gainprm[0] = kp;
          pact->biasprm[1] = -kp;
          pact->biasprm[2] = -kv;
        } else if (
          joint_info.command_interfaces.at(0).name == "velocity")
        {
          joint_control_methods_[j] |= VELOCITY;
          mjs_setString(pact->name, (joint_name + "_vel").c_str());
          // https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-velocity
          auto it = joint_info.parameters.find("d");
          if (it != joint_info.parameters.end()) {
            kv = std::stod(it->second);
          } else {
            kv = 1;
          }
          pact->gainprm[0] = kv;
          pact->biasprm[2] = -kv;
        } else if (
          joint_info.command_interfaces.at(0).name == "effort")
        {
          joint_control_methods_[j] |= EFFORT;
          mjs_setString(pact->name, (joint_name + "_eff").c_str());
          // https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-motor
          pact->gainprm[0] = 1;
        }
        auto act_name = mjs_getString(pact->name);
        std::cout << "Added actuator " << act_name << std::endl;
      } else if (joint_info.command_interfaces.size() == 2 &&
        joint_info.command_interfaces.at(0).name == "position" &&
        joint_info.command_interfaces.at(1).name == "velocity")
      {
        joint_control_methods_[j] |= POS_VELOCITY;
        mjs_setString(pact->name, (joint_name + "_pos_vel").c_str());
        // https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-motor
        pact->gainprm[0] = 1;

        auto it = joint_info.parameters.find("p");
        if (it != joint_info.parameters.end()) {
          kp = std::stod(it->second);
        } else {
          kp = 1.;
        }
        it = joint_info.parameters.find("d");
        if (it != joint_info.parameters.end()) {
          kv = std::stod(it->second);
        } else {
          kv = 0.;
        }
        stiff[j] = kp;
        damp[j] = kv;

        auto act_name = mjs_getString(pact->name);
        std::cout << "Added actuator " << act_name << " with kp,kv " << kp << ", " << kv <<
          std::endl;
      }
      // TODO: add something useful here
      mjs_setString(pact->info, joint_name.c_str());
    }
  }

  // get update rate
  auto it = hw_info.hardware_parameters.find("update_rate");
  double update_rate = 100;
  if (it != hw_info.hardware_parameters.end()) {
    update_rate = std::stod(it->second.c_str());
  } else {
    std::cout << "No update rate found, using default value of 100 Hz" << std::endl;
  }
  auto update_period =
    std::chrono::duration<double>(1.0 / update_rate);
  spec->option.timestep = update_period.count();

  m = mj_compile(spec, nullptr);
  if (!m) {
    mju_error("Load model error");
    return 1;
  }

  // Only compiled model can be written
  std::string xml_out = model_xml + "changed.xml";
  if (!mj_saveXML(spec, xml_out.c_str(), error, 1000)) {
    mju_error("Save model error: %s", error);
    return 1;
  }
  std::cout << "Model adapted and saved to: " << xml_out << std::endl;

  // Set initial state
  d = mj_makeData(m);
  if (static_cast<int>(init_val.size()) != m->nq) {
    mju_warning(
      "Something went wrong while parsing the initial state, size is %d but expected %d",
      init_val.size(), m->nq);
  } else {
    mju_copy(d->qpos, init_val.data(), m->nq);
  }

  // Initialize buffers for ros2_control.
  pos_state.resize(m->nq);
  vel_state.resize(m->nq);
  eff_state.resize(m->nq);
  pos_cmd.resize(m->nu, std::numeric_limits<double>::quiet_NaN());
  vel_cmd.resize(m->nu, std::numeric_limits<double>::quiet_NaN());
  eff_cmd.resize(m->nu, std::numeric_limits<double>::quiet_NaN());

  // Start where we are
  syncStates();
  state_mutex.unlock();

  // Connect our specific control input callback for MuJoCo's engine.
  mjcb_control = MuJoCoSimulator::controlCB;

  std::cout << "wait for first write: " << first_write << std::endl;
  while (first_write == false) {}
  std::cout << "start simulation: " << first_write << std::endl;
  std::cout << "with update rate: " << update_rate << std::endl;
  std::cout << "with update period: " << update_period.count() << std::endl;

  // Simulate in realtime
  while (true) {
    auto start = std::chrono::system_clock::now();

    mj_step(m, d);

    // Provide fresh data for ros2_control
    state_mutex.lock();
    syncStates();
    state_mutex.unlock();

    auto end = std::chrono::system_clock::now();

    // Sleep for a while
    std::this_thread::sleep_for(update_period - (end - start));
  }

  return 0;
}

void MuJoCoSimulator::read(
  std::vector<double> & pos, std::vector<double> & vel,
  std::vector<double> & eff)
{
  // Realtime in ros2_control is more important than fresh data exchange.
  if (state_mutex.try_lock()) {
    pos = pos_state;
    vel = vel_state;
    eff = eff_state;
    state_mutex.unlock();
  }
}

void MuJoCoSimulator::write(
  const std::vector<double> & pos, const std::vector<double> & vel, const std::vector<double> & eff)
{

  if (std::none_of(
      pos.begin(), pos.end(),
      [](double i) {return std::isnan(i);}) ||
    std::none_of(
      vel.begin(), vel.end(),
      [](double i) {return std::isnan(i);}) ||
    std::none_of(
      eff.begin(), eff.end(),
      [](double i) {return std::isnan(i);}))
  {
    first_write = true;
  }
  // Realtime in ros2_control is more important than fresh data exchange.
  if (command_mutex.try_lock()) {
    pos_cmd = pos;
    vel_cmd = vel;
    eff_cmd = eff;
    command_mutex.unlock();
  }
}

void MuJoCoSimulator::syncStates()
{
  for (auto i = 0; i < m->nq; ++i) {
    pos_state[i] = d->qpos[i];
    vel_state[i] = d->qvel[i]; // d->actuator_velocity[i];
    eff_state[i] = d->actuator_force[i];
  }
}

}  // namespace mj_ros2_control
