# Mapping and Localization

Install Dependencies:
- ``$ sudo apt-get install ros-noetic-dynamixel-sdk``
- ``$ sudo apt-get install ros-noetic-turtlebot3-msgs``
- ``$ sudo apt-get install ros-noetic-turtlebot3``

Use the turtlebot3_world and the burger model for the turtlebot3
- ``$ roslaunch turtlebot3_gazebo turtlebot3_world.launch``
- ``$ export TURTLEBOT3_MODEL=burger``

Add command line to .bashrc
- ``$ vim ~/.bashrc``
- ``$ export TURTLEBOT3_MODEL=burger``
- ``$ :wq``
- ``$ source ~/.bashrc``

Setup RViz visualization tool
- ``$ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch``

## Teleoperating the robot with the keyboard
- Use a teleoperation program that controls the robot by keyboard strokes, which will change the velocity values of the robot: ``$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch``
- Check different topics. There are multiple options for rostopic command-line tool, for example:  list, echo, hz, pub: ``$ rostopic echo /cmd_vel``
- Gather information on the nodes that are active and how they are exchanging information through topics: ``$ rosrun rqt_graph rqt_graph``

![rqt_graph](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/2_Mapping_and_Localization/assets/Figure_1.png)

## Creating a map
- With the gazebo world launched (turtlebot3\_world) and the teleoperation program running (make sure to terminate the RViz terminal): ``$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping``
- If your gmapping does not work, it is highly likely the installation is not correct: ``$ sudo apt-get install ros-noetic-slam-gmapping``

![Nodes & Topics](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/2_Mapping_and_Localization/assets/Figure_2.png)

- Teleoperate the robot around the room. When you have finished tracing the surroundings to complete the map, you can save it by running the following command in the terminal after finishing the gmapping launch: ``$ rosrun map_server map_saver -f ~/map``

<div align=center><img src="https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/2_Mapping_and_Localization/map/map.png"/></div>

## Navigating the map. 2D PoseEstimate and 2DNavGoal
- Install Dependencies: ``sudo apt-get install ros-noetic-dwa-local-planner``
- Work with the turtlebot3\_world and the teleoperation program (make sure to close the RViz window it will be relaunched), we will launch a navigation file: ``$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml``
- Start moving the Robot around the room using the keyboard teleop.

![Sensor Measurement](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/2_Mapping_and_Localization/assets/Figure_3.png)

- Re-initialize robot location on the map.

![Re-initialize](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/2_Mapping_and_Localization/assets/Figure_5.png)

- Another feature in the RViz GUI, is the option to send Navigation goals to the robot. At this point, the teleoperation program needs to be terminated. You can command the robot by pressing the button 2DNavGoal and clicking on the map where an arrow will appear and release the mouse to provide the orientation. You can observe the planned trajectory and the robot moving towards this new location.

- Another way to provide navigation goals is by publishing directly into the goal topic: ``$ rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: <value>, y: <value>, z: <value>}, orientation: {w: <value>}}}'``

- Add an obstacle (insert the pillar you previously created) You now have both static obstacles and a dynamic (new) obstacle that isn’t part of your map. 

![Nodes & Topics](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/2_Mapping_and_Localization/assets/Figure_6.png)

- Navigate robot to a new location using 2DNav Goal without the pillar.
[![Navigation](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/2_Mapping_and_Localization/assets/Figure_7.png)](https://drive.google.com/file/d/1bKYwpVqsLr12XZPvQHSxt5w8SbOVYWXH/view?usp=sharing)

- Navigate robot to a new location using 2DNav Goal that avoids the inserted pillar.
[![Navigation](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/2_Mapping_and_Localization/assets/Figure_8.png)](https://drive.google.com/file/d/13RZZ2YRex00Ja_xRPLdAo2WX7N88LAna/view?usp=sharing)