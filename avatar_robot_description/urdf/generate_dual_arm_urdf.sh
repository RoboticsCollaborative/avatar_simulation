#! /usr/bin/env bash

# Generate URDF with mimic
rosrun xacro xacro $1/urdf/dual_panda_with_gripper.urdf.xacro mimic:=true > $1/urdf/dual_panda_with_gripper.urdf
# Generate URDF without mimic
rosrun xacro xacro $1/urdf/dual_panda_with_gripper.urdf.xacro mimic:=false > $1/urdf/dual_panda_with_gripper_nomimic.urdf

echo "[INFO] URDF saved"


