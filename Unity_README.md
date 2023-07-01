## How to use this model in Unity
### Import URDF from ROS
If you haven't import the URDF from ROS into Unity, run the following command on the ROS machine:
```
roslaunch avatar_robot_description publish_description_avatar.launch
```
In Unity, select `RosBridgeClient`->`Transfer URDF from ROS`, fill in the correct rosbridge server IP (ROS machine IP) and select `Read Robot Description`. This will generate the robot model (a game object `avatar`) in Unity for you.

### Publish robot states from rosbag data
On ROS machine, start robot state publisher:
```
roslaunch avatar_robot_description test_dual_panda.launch mimic:=false real_robot:=true
```

Start rosbridge server:
```
roslaunch rosbridge_server rosbridge_websocket.launch
```

Play rosbag data:
```
rosbag play [ws]/src/avatar_simulation/data/dice2_2023-06-20-20-55-48.bag
```
You can press `space` to pause/unpause the bag play.

### Play Unity scene
Select `ROSConnector` in the Hierarchy in Unity, enbale `Subscribe joint states` for Joint State Patcher.

Click play.

**You can move around the XROrigin to adjust the VR camera pose**