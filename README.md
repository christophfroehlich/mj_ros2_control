# mj_ros2_control

## known limitations
URDF body needs `<collision>` to create a geom in mujoco, or add 
  <mujoco>
    <compiler balanceinertia="true" discardvisual="false"/>
  </mujoco>