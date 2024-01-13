# A ROS Based Simulation and Control Framework for In-Orbit Multi Arm Robot Assembly Operations
A ROS-Gazebo based simulation showcasing the required in-orbit operation objectives as assembly and locomotion framework for a multi-manipulator system.

Developed as part of MSc Group Design Project titled "A ROS Based Simulation and Control Framework for In-Orbit Multi Arm Robot Assembly Operations" 
by ***Sairaj R Dillikar (MSc Robotics 2022-23, Cranfield University, United Kingdom)***.

The team members include: ***Saksham Bhadani, Irene Cotrina de los Mozos, Omkar N Pradhan (MSc Robotics 2022-23, Cranfield University, United Kingdom)***

Simulation videos of this project can be accessed by clicking [here](http://tinyurl.com/Assembler-MARIO).

## Table of Contents

- [Build the Package](#build-the-package)
    - [Requirements](#requirements)
    - [Modifications to be made in the SRS Spawning Package](#modifications-to-be-made-in-the-srs-spawning-package)
- [Demonstration](#demonstration)
    - [Launching the Simulation](#launching-the-simulation)
- [Citation](#citation)
    - [Repository](#repository)
- [Contact](#contact)

## Build the Package

````
mkdir -p mario_ws/src
cd mario_ws/src
catkin_init_workspace
git clone https://github.com/sairajdillikar/A-ROS-Based-Simulation-and-Control-Framework-for-In-Orbit-Multi-Arm-Robot-Assembly-Operations.git
cd ..
catkin build
````

### Requirements

- [Ubuntu 2020.04](https://releases.ubuntu.com/focal/)
- [ROS Noetic Ninjemys 8](https://wiki.ros.org/noetic)
- [Gazebo v11.11](https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)
- [Moveit! 1.0](https://moveit.ros.org/install/) for ROS Neotic
- [Python 3.7](https://www.python.org/downloads/release/python-370/)
- [*ros_link_attacher*](https://github.com/pal-robotics/gazebo_ros_link_attacher) Plugin (already included within this repository ([gazebo_ros_link_attacher](gazebo_ros_link_attacher)))

## Modifications to be made in the SRS Spawning Package

Before launching the gazebo world, navigate through the packages to find the `srs_modules_description/meshes/srs_module.sdf` file and make sure to add your PC **`$USERNAME`** within the `<uri>` tag.

    <uri>/home/ $USERNAME /mario_ws/src/srs_modules_description/meshes/base_link.stl</uri>

## Demonstration

Before launching a simulation, any new terminal must be always sourced with,

    cd mario_ws
    source devel/setup.bash

Use the above two commands when using a new terminal, as executing simulation for each different locomotion requires only two main commands: 
1. For launching the simulation (`roslaunch`).
2. To run the script with its respective control algorithm (`rosrun`).

### Launching the Simulation:

Terminal-1: 
    
    roslaunch moveit_mario_so2 main_execution.launch

Terminal-2:

    rosrun moveit_mario_so2 main_execution_sequence.py

![Serial (a) Locomotion](resources/serial_motion_1.gif)

## Citation

If you use the Motion Planning architecture as part of a publication or utilise in a project development, please use the Bibtex below as a citation,

### Repository:
```bibtex
@misc{mario_assembler_git,
  author       = {Dillikar, Sairaj.R. and Bhadani, Saksham and Cotrina de los Mozos, Irene and Pradhan, Omkar.N. and Felicetti, Leonard and Upadhyay, Saurabh and Tang, Gilbert},
  booktitle    = {GitHub repository},
  publisher    = {GitHub},
  title        = {A ROS Based Simulation and Control Framework for In-Orbit Multi Arm Robot Assembly Operations},
  month        = {Oct},
  year         = {2023},
  url          = {https://github.com/sairajdillikar/A-ROS-Based-Simulation-and-Control-Framework-for-In-Orbit-Multi-Arm-Robot-Assembly-Operations}
}
```

## Contact

For questions, suggestions, or collaborations, feel free to reach out to the project author at [sairajdillikar@gmail.com](mailto:sairajdillikar@gmail.com).
