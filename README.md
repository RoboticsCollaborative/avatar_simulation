# avatar_simulation

## Usage

### Gripper test
Run 
`roslaunch avatar_gripper_description test_gripper.launch`
to visualize the gripper in Rviz.

![](https://github.com/RoboticsCollaborative/avatar_simulation/blob/master/avatar_gripper_sim.gif)

### Single arm test
Run

`roslaunch avatar_robot_description test_single_arm.launch`

to visualize one single arm with gripper as the end-effector in Rviz.


### Play robot trajectory from rosbag

We recorded two rosbags stored under data folder. To play the trajectory and visualize it in Rviz, run:

`rosbag play [path/to/the/bag] --clock`
 
`roslaunch avatar_robot_description test_single_arm.launch real_robot:=true`

### URDF

The `avatar_gripper_3f.urdf` is automatically generated from `avatar_gripper_3f.xacro` with [xacro](http://wiki.ros.org/xacro).