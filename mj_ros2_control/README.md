# Cmj_ros2_control

## Build and install
We use MuJoCo in [headless mode](https://mujoco.readthedocs.io/en/latest/programming.html?highlight=headless#using-opengl)
and don't need OpenGL-related dependencies.

1. Download MuJoCo's pre-built [library package](https://github.com/deepmind/mujoco/releases/) and extract that somewhere.
It's ready-to-use and we will just point to it during the build.
   ```bash
   cd $HOME
   wget https://github.com/deepmind/mujoco/releases/download/3.0.0/mujoco-3.0.0-linux-x86_64.tar.gz
   tar -xf mujoco-3.0.0-linux-x86_64.tar.gz
   ```

3. Switch to the *root* of your ROS2 workspace and build the package (*standalone*) with
   ```bash
   colcon build --cmake-args "-DMUJOCO_DIR=$HOME/mujoco-3.0.0" --packages-select mj_ros2_control
   ```
