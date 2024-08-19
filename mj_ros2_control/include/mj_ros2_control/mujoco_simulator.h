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
/*!\file    mujoco_simulator.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2022/02/01
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "mujoco/mujoco.h"

namespace mj_ros2_control
{
template<class ENUM, class UNDERLYING = typename std::underlying_type<ENUM>::type>
class SafeEnum
{
public:
  SafeEnum()
  : mFlags(0) {}
  explicit SafeEnum(ENUM singleFlag)
  : mFlags(singleFlag) {}
  SafeEnum(const SafeEnum & original)
  : mFlags(original.mFlags) {}
  ~SafeEnum() = default;

  SafeEnum & operator=(const SafeEnum & original) = default;
  SafeEnum & operator|=(ENUM addValue) {mFlags |= addValue; return *this;}
  SafeEnum operator|(ENUM addValue) {SafeEnum result(*this); result |= addValue; return result;}
  SafeEnum & operator&=(ENUM maskValue) {mFlags &= maskValue; return *this;}
  SafeEnum operator&(ENUM maskValue) {SafeEnum result(*this); result &= maskValue; return result;}
  SafeEnum operator~() {SafeEnum result(*this); result.mFlags = ~result.mFlags; return result;}
  explicit operator bool() {return mFlags != 0;}

protected:
  UNDERLYING mFlags;
};

// Methods used to control a joint.
enum ControlMethod_
{
  NONE      = 0,
  POSITION  = (1 << 0),
  VELOCITY  = (1 << 1),
  EFFORT    = (1 << 2),
  POS_VELOCITY    = (1 << 3),
};

typedef SafeEnum<enum ControlMethod_> ControlMethod;
/**
 * @brief MuJoCo's physics engine with rendering and basic window mouse interaction
 *
 * It's implemented as a singleton class, which circumvents the problem of
 * using global function pointers for the control callback.
 *
 * User code interfaces this class by getting an instance and calling static
 * functions on it.  It's designed to run with an independent simulation rate,
 * disjoint from ros2_control in a separate thread.
 *
 */
class MuJoCoSimulator
{
private:
  // Private constructor to enforce singleton pattern
  MuJoCoSimulator();

  // Lock the mutex for these calls
  void syncStates();

public:
  // Modern singleton approach
  MuJoCoSimulator(const MuJoCoSimulator &) = delete;
  MuJoCoSimulator & operator=(const MuJoCoSimulator &) = delete;
  MuJoCoSimulator(MuJoCoSimulator &&) = delete;
  MuJoCoSimulator & operator=(MuJoCoSimulator &&) = delete;

  // Singleton instance accessor
  // Use this in ROS2 code
  static MuJoCoSimulator & getInstance()
  {
    static MuJoCoSimulator simulator;
    return simulator;
  }

  // MuJoCo data structures
  mjModel * m = NULL;  // MuJoCo model
  mjData * d = NULL;   // MuJoCo data

  // Buffers for data exchange with ros2_control
  std::vector<double> pos_cmd;
  std::vector<double> vel_cmd;
  std::vector<double> eff_cmd;
  std::vector<double> pos_state;
  std::vector<double> vel_state;
  std::vector<double> eff_state;
  std::vector<double> stiff;  // Proportional gain
  std::vector<double> damp;   // Derivative gain

  /// \brief vector with the control method defined in the URDF for each joint.
  std::vector<ControlMethod> joint_control_methods_;

  // Safety guards for buffers
  std::mutex state_mutex;
  std::mutex command_mutex;

  // Control input callback for the solver
  // Static callback function required by the C library
  static void controlCB(const mjModel * m, mjData * d);
  // Instance method that contains the actual implementation
  void controlCBImpl(const mjModel * m, mjData * d);

  // Call this in a separate thread
  static int simulate(
    const std::string & model_xml,
    const hardware_interface::HardwareInfo & hw_info);
  int simulateImpl(
    const std::string & model_xml,
    const hardware_interface::HardwareInfo & hw_info);

  // Non-blocking
  void read(std::vector<double> & pos, std::vector<double> & vel, std::vector<double> & eff);
  void write(
    const std::vector<double> & pos, const std::vector<double> & vel,
    const std::vector<double> & eff);
};

}  // namespace mj_ros2_control
