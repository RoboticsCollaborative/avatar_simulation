#! /usr/bin/env bash

echo $1

# Generate URDF with mimic
rosrun xacro xacro $1/urdf/avatar_gripper_3f_example.xacro mimic:=true prefix:=right motor_base_attach_to:=right_gripper_base > $1/urdf/avatar_gripper_right.urdf
# Generate URDF without mimic
rosrun xacro xacro $1/urdf/avatar_gripper_3f_example.xacro mimic:=false prefix:=right motor_base_attach_to:=right_gripper_base > $1/urdf/avatar_gripper_right_nomimic.urdf

echo "[INFO] URDF saved"


