# Sensors in ROS
This is a simple project that introduces basic knowledge about how to simulate robot in ROS.

## Gazebo Intro
Gazebo is a high-fidelity 3D simulator stemming from the need to simulate robots in various environments and conditions.

Try command to see the example：``$ roslaunch gazebo_ros empty_world.launch world_name:=worlds/cafe.world``


## Spawning a robot in Gazebo
This method uses a small python script called spawn_model to make a service call request to the
gazebo_ros ROS node (named simply "gazebo" in the rostopic namespace) to add a custom URDF
into Gazebo. The spawn_model script is located within the gazebo_ros package. Open a new
termianl and use this script in the following way:

- Install through catkin
- ``$ cd ~/catkin_ws/src/``
- ``$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_ms```gs.git``
- ``$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git``
- ``$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git``
- ``$ sudo apt install ros-noetic-turtlebot3-bringup``
- ``$ cd ~/catkin_ws && catkin_make``
- Install through repository in ros orginal path
- `` $ sudo apt-get install ros-noetic-turtlebot*``
- ``$ roscd turtlebot3_description/urdf/``
- ``$ rosrun gazebo_ros spawn_model -urdf -file turtlebot3_burger.urdf.xacro -x 0 -y 0 -z 1 -model TURTLEBOT3_MODEL``

## Creating object models in Gazebo
You can add additional objects and shapes into your simulated environment.
