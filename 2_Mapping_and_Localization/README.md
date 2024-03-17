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
