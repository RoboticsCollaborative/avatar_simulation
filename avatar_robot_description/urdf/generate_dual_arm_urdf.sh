#! /usr/bin/env bash

# Generate URDF with mimic
rosrun xacro xacro $HOME/avatar_ws/src/avatar_simulation/avatar_robot_description/urdf/dual_panda_with_gripper.urdf.xacro mimic:=true > $HOME/avatar_ws/src/avatar_simulation/avatar_robot_description/urdf/dual_panda_with_gripper.urdf
# Generate URDF without mimic
rosrun xacro xacro $HOME/avatar_ws/src/avatar_simulation/avatar_robot_description/urdf/dual_panda_with_gripper.urdf.xacro mimic:=false > $HOME/avatar_ws/src/avatar_simulation/avatar_robot_description/urdf/dual_panda_with_gripper_nomimic.urdf

echo "[INFO] URDF saved"


