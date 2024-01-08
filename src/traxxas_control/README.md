# How do you create a workspace?
```bash
cd ros/src
catkin_init_workspace
cd ../
catkin_make
```


## What else do we need to install?
```bash
sudo apt install ros-kinetic-joy ros-kinetic-joystick-drivers
sudo apt install ros-kinetic-rosserial-arduino
sudo apt install ros-kinetic-rosserial
```

## Creating the package
```bash
cd ros/src
catkin_create_pkg traxxas_control rospy
```

### Structure of a typical ROS package
* scripts (python executables)
* src (C++ source files)
* msg (for custom message definitions)
* srv (for service message definitions)
* include -> headers/libraries that are needed as dependencies
* config -> configuration files
* launch -> provide a more automated way of starting nodes

Other folders may include

* urdf (Universal Robot Description Files)
* meshes (CAD files in .dae (Collada) or .stl (STereoLithography) format)
* worlds (XML like files that are used for Gazebo simulation environments)

#### Creating a `traxxas_control` node

```bash
cd ros/src/traxxas_control
mkdir scripts
cd scripts
```
