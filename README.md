# swarmsim
ROS2 package, bridging uw_swarmsim to the ROS2 envrioment. 
Make sure to have uw_swarmsim installed: [uw_swarmsim link](https://github.com/save-xx/uw_swarmsim)
Current version: Humble

## Setup
The Package is supposed to work as an extension of uw_simulator for ROS2.
Make sure to have the uw_simulator folder added to PYTHONPATH.

## Install
The installation assumes an existing ROS2 Humble installation 
in the src folder of the ros workspace clone this repository. The build the package.

## Usage
To activate the simulator launch the node swarmsim  
```bash
ros2 run swarmsim swarmsim
```

The swarmsim node can be used in 3 modes, based on the launching parameter `mode`:
- `ue5`: bridges Unreal Engine 5, swarmsim core and ROS togheter. it provides the API required by the UE5 emulator. Alwaysy runs in real time at 60fps.
- `rt`: advances the simulation at real speed time (if computation allows), based on the indicated fps rate.
- `step`: advances one step of simulation at the time. A subscriber `sim_step` of Type example_interfaces/msg/Bool with value True, awaits a topic for each simulation step. Based on the `sim_step` frequency, the effective simulation can go either slower or faster than Real Time
