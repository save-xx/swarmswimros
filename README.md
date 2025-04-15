# swarmswimros
ROS2 package, bridging swarmswimros to the ROS2 envrioment. 
Make sure to have uw_swarmsim installed: [uw_swarmsim link](https://github.com/save-xx/uw_swarmsim)
Current version: Humble

## Setup
The Package is supposed to work as an extension of uw_simulator for ROS2.
Make sure to have the uw_simulator folder added to PYTHONPATH.

## Install
The installation assumes an existing ROS2 Humble installation 
in the src folder of the ros workspace clone this repository. The build the package.

For building the package separately, from your ros workspace folder:

```bash
colcon build --symlink-install --packages-select swarmsim
```

## Usage
To activate the simulator launch the node swarmsim  
```bash
ros2 run swarmsim swarmsim
```
or by specifing the mode:
```bash
ros2 run swarmsim swarmsim --ros-args -p mode:=rt
```

The swarmsim node can be used in 3 modes, based on the launching parameter `mode`:
- `ue5`: bridges Unreal Engine 5, swarmsim core and ROS togheter. it provides the API required by the UE5 emulator. Alwaysy runs in real time at 60fps.
- `rt`: advances the simulation at real speed time (if computation allows), based on the indicated fps rate.
- `step`: advances one step of simulation at the time. A subscriber `sim_step` of Type example_interfaces/msg/Bool with value True, awaits a topic for each simulation step. Based on the `sim_step` frequency, the effective simulation can go either slower or faster than Real Time

## Topics
List of Publisher/Subscribers. Design note: Proper implementation would require fo the generation of custom message types. This however imposes the uses of a c++ package and the need of building such message type to run the simulation as well as analysing the bags. 
For the current version we choose the approach of adopting existing message types, avaiable as default with the current ROS version.

### General
During Each run the node generate a limited number of global topics. Those topics manage simulation proprieties

**Publishers**  
- /clock **[rosgraph_msgs/msg/Clock]** : defalut clock message used in 'use_sim_time'.  

**Subscribers**  
- /sim_step **[example_interfaces/msg/Bool]** (optional) : message required to move the simulation foward one tick in `step` mode. Advance one tick if data=`True`
- /acoustic_send **[sensor_msgs/msg/CompressedImage]** (optional): if Acoustic is included inlet of all initiated acoustic communications. 
    - header.frame_id = Sender name 
    - data = byte array of the transmitted message

### Agents Topics
For each agent a namespace is created with the name of the agent (ex: '/A01, /A02' etc..)
Associated to each agent a set of publishers subscribers are created.

**Publishers** 
- /gt **[geometry_msgs/msg/TwistStamped]**: ground truth, real position and heading of the agent. 
    - linear: coordinates of agent (x,y,z NED) in the world
    - angular.z: heading in degrees to the North
- /depth **[geometry_msgs/msg/Vector3Stamped]**: Estimated  vertical position of the robot (as used by the depth control). the value is stored in vector.z 
- /heading **[geometry_msgs/msg/Vector3Stamped]**: Estimated heading in degrees of the robot (as used by the heading control). the value is stored in vector.z 
- /echo **[sensor_msgs/msg/Range]** (optional, UE5 mode): Distance of the robot from the seabed. Value stored in range
- /view **[sensor_msgs/msg/Image]** (optional, UE5 mode): Image topic containing the syntetic view of the frontal camera
- /NNDetection **[nav_msgs/msg/Path]** (optional): Detection of the emulated NN detector and estimation of other agents relative position. Note that detector is independent and uncorrelated with UE5. 
- /acoustic **[sensor_msgs/msg/CompressedImage]** (optional): Recived acoustic messages.
- /ac_ranges **[sensor_msgs/msg/JointState]** (optional): Metatadas relative to recived acousic messages, inculding estimated range and Doppler shift of the message. 

**Subscribers**  
- /cmd **[geometry_msgs/msg/TwistStamped]**: update command to the agent. Meaning of each field is determined by the control mode acting on the agent. 
    - twist/linear/x, - twist/linear/y: In position control, waypoint coordinates. In velocity control, body velocity vector (u,v). In force control, Force vector in body axis.
    - twist/linear/z: In direct control, heave velocity. Otherwise taget depth.
    - twist/angular/z: In yawrate control, the yawrate in degrees/second. Otherwise, desired heading in degrees. 
