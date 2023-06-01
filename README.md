# Impact of Communication Latency on Robotic Push Actions

Implementation of linear push actions with UR5e, Robotiq 2F-85 gripper and custom cube using Ubuntu 22.04, ROS2 Humble and Gazebo 11

## External Packages
(to be included into the workspace along the packages from this repository)
- ROS2 UR description package (ur_description): https://github.com/UniversalRobots/Universal_Robots_ROS2_Description
- IOC-UPC inverse kinematics library for UR robots (kinenik): https://gitioc.upc.edu/robots/kinenik

## Packages in this repository
- main_pkg: setup of the gazebo environment with robot, gripper and cubes, and velocity controller, using modified launch file from https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation, and modified multi-object launch mechanism by TheConstruct: https://bitbucket.org/theconstructcore/box_bot/src/foxy/box_bot_description/launch/
- experiments: Two experiments for pushing a cube in a straight line or to change its rotation
- custom_interfaces: Includes the custom messages and services
- robotiq_2f_model: model from https://github.com/beta-robots/robotiq/tree/master/robotiq_2f_model wrapped in ROS2 package

## Latency distributions
The [latency distributions](https://github.com/DominikUrbaniak/Latency_Push_Actions/tree/main/latency_distributions) of Ethernet and private 4G, private 5G, 5G URLLC, and ideal and loaded Wi-Fi 5 networks are owned by Aalborg University, Denmark. The usage of this data requires the citation of two papers¹.

## Run experiment

- *ros2 launch main_pkg main.launch.py*
- *ros2 run experiments push_rot_gazebo 0.05*
- *ros2 run experiments push_rot_main_fresh 0.05 private5g 0.0001 1 0 1*

##

¹ References for latency distributions:

I. Rodriguez, R.S. Mogensen, A. Schjørring, M. Razzaghpour, R. Maldonado, G. Berardinelli, R. Adeogun, P.H. Christensen, P. Mogensen, O. Madsen, C. Møller, G. Pocovi, T. Kolding, C. Rosa, B. Jørgensen, and S. Barbera, 5G Swarm Production: Advanced Industrial Manufacturing Concepts enabled by Wireless Automation, IEEE Communications Magazine, vol. 59, no. 1, pp. 48-54, January 2021. [[DOI](https://doi.org/10.1109/MCOM.001.2000560)] [[pdf](https://vbn.aau.dk/files/402417271/AAUindustrial_5Gswarm_irl.pdf)]

I. Rodriguez, R.S. Mogensen, A. Fink, T. Raunholt, S. Markussen, P.H. Christensen, G. Berardinelli, P. Mogensen, C. Schou, and O. Madsen, An Experimental Framework for 5G Wireless System Integration into Industry 4.0 Applications, Energies, vol. 14, no. 15, 4444, July 2021. [[DOI](https://doi.org/10.3390/en14154444)] [[pdf](https://www.mdpi.com/1996-1073/14/15/4444/pdf)]
