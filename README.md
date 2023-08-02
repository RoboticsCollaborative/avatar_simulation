# avatar_simulation

## Usage

### Gripper test
Run 
```
roslaunch avatar_gripper_description test_gripper.launch
```
The URDF with mimic tag is used by default, to use non-mimic version, run
```
roslaunch avatar_gripper_description test_gripper.launch mimic:=false
```

to visualize the gripper in Rviz.

![](https://github.com/RoboticsCollaborative/avatar_simulation/blob/master/avatar_gripper_sim.gif)

### Single arm test
Run

```
roslaunch avatar_robot_description test_single_arm.launch
```

to visualize one single arm with gripper as the end-effector in Rviz.

The URDF with mimic tag is used by default, to use non-mimic version, run
```
roslaunch avatar_gripper_description test_single_arm.launch mimic:=false
```

**Dual arm**: 

```
roslaunch avatar_robot_description test_dual_arm.launch mimic:=false
```


### Play robot trajectory from rosbag

We recorded two rosbags stored under data folder. To play the trajectory and visualize it in Rviz, run:

```
rosbag play [path/to/the/bag] --clock
```

**Single arm**:

```
roslaunch avatar_robot_description test_single_arm.launch real_robot:=true
```

**Dual arm**: 

```
roslaunch avatar_robot_description test_dual_arm.launch mimic:=false real_robot:=true
```

![](https://github.com/RoboticsCollaborative/avatar_simulation/blob/master/avatar_robot_sim.gif)

### URDF

The URDF file will be automatically generated under the `urdf` folder.
