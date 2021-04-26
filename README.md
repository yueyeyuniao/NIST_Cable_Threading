# Benchmarking Autonomous Manipulation of Deformable Linear Objects in Constrained Spaces (RAL 2021) [under review]
## [Paper link](https://drive.google.com/file/d/18-0WE0f70hS8QTbjbHv9Cg_iiWI2REK8/view?usp=sharing)
## [Video link](https://drive.google.com/file/d/1_ROtOzjHuf5BDvUOtFnqb7buURaLELdB/view?usp=sharing)

## System overview:
![alt-text](https://github.com/yueyeyuniao/NIST_Cable_Threading/blob/main/media/workspace_annotated.PNG)<br/>


## System requirement:
#### ROS Kinectic
#### [Darknet_ros](https://github.com/leggedrobotics/darknet_ros)
#### Opencv
#### PCL
#### [Kinova ros_kortex package](https://github.com/Kinovarobotics/ros_kortex)
#### [Kinova ros_kortex_vision package](https://github.com/Kinovarobotics/ros_kortex_vision)
#### [Trajop_ros](https://github.com/ros-industrial-consortium/trajopt_ros)

## Hardware requirement:
#### Kinova gen3 arm mounted with RGBD camera

## Introduction:
#### /darknet_ros folder includes all the configurates and weights for tube and cable detection
#### /nist folder includes all the launch file, source codes, and service files
#### /research folder includes all the configuration and launch files for integrating Trajopt with Gen3 arm
#### /trajopt_ros folder includes the trajopt (optimized motion planner) in ROS

## Commands
####launch robot and camera
#####(roslaunch nist nist.launch)
####launch trajopt service node
#####(roslaunch research gen3_trajopt_node.launch)
####insert cable modeling (servoing) (insert to tube 1)
#####(rosrun nist modeling_cable_nist_insert)
####insert cable modeling (servoing) (insert to final holder)
#####rosrun nist modeling_cable_nist_insert_final_holder
####run trajopt - go the start position
#####(rosrun nist nist_start __ns:=my_gen3)
####run color detection node - kinova camera
#####(rosrun nist color_detection_blue_kinova_node)
####run modeling cable node for pushing cable
#####(rosrun nist modeling_cable_gen3_nist_node)
####run yolo detection to detect the board and tubes
#####(roslaunch darknet_ros darknet_ros_NIST_board.launch)
####run yolo detection to detect the cable
#####(roslaunch darknet_ros darknet_ros_NIST_cable.launch)
####run plane detection to detect the board plane
#####(rosrun nist plane_segmentation)
####run trajopt - run the task
#####(rosrun nist nist_run __ns:=my_gen3)


## Copyright: 
### This work was developed at the [RIVeR Lab, Northeastern University](http://robot.neu.edu/) 



