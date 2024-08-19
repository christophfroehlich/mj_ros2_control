# mj_ros2_control_demos

## Getting started
In a sourced terminal, run
```bash
ros2 launch mj_ros2_control_demos r6bot.launch.py
```
The *launch* part will start a simulated world with a generic robot model.
You can call
```bash
ros2 control list_controllers
```
to get a list of controllers currently managed by the `controller_manager`.