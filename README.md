# mission_coordination_lab_report

The objective of this project is to navigate three simulated robots in a ROS 1 and Gazebo
environment from their starting positions to their respective flags (robot1 to flag1, robot2
to flag2, and robot3 to flag3) while avoiding collisions. The goal is to reach the flags as
quickly as possible using robots equipped with ultrasonic sensors (range: 5 meters),
motorized wheels, and real-time pose awareness (position and orientation).

### Step 1 : Open a terminal, navigate to a new folder, and execute the following commands:

 cd ~/catkin_ws/src && git clone https://github.com/KTBE/Mission_Coordination_project.git

 cd ~/catkin_ws && catkin_make && source ~/catkin_ws/devel/setup.bash

### Step 2 : Run the following commands sequentially in the same terminal to make the agent.py file executable :

 $cd / home / user / catkin_ws /src/ Mission_Coordination_project /
 evry_project_strategy / nodes

 $chmod +x agent .py

 $cd ~

### Step 3 : Gazebo simulation:

Open a new terminal and run the following code to run the simulation : 

roslaunch evry_project_description simu_robot.launch

### Step 4 : Run the python script containing the strategy by using this code :

roslaunch evry_project_strategy agent.launch

roslaunch evry_project_strategy agent_strategy1.launch

roslaunch evry_project_strategy agent_with_pid.launch

roslaunch evry_project_strategy agent_with_timing.launch

