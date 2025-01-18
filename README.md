# mission_coordination_lab_report
## Environment Set Up
 $cd / home / user / catkin_ws /src/ Mission_Coordination_project /
 evry_project_strategy / nodes

 $chmod +x agent .py

 $cd ~


## GAZEBO AND THE STRATEGY NODE
first terminal

$roslaunch evry_project_description simu_robot . launch
In another terminal

second terminal

$roslaunch evry_project_strategy agent . launch nbr_robot :=1
# Lab2: simple and non-robust strategy
