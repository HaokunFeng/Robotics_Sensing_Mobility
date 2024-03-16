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
- ``$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git``
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

![Figure](https://github.com/HaokunFeng/Robotics_Sensing_Mobility/blob/main/1_Sensors_in_ROS/assets/Figure_4.png)

## Setting up the working environment
- elect the turtlebot3 model to be burger and use the ’roslaunch’ command to spawn a gazebo
world with a turtlebot3 in it ``$ export TURTLEBOT3_MODEL=burger`` & ``$ roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch``
- Open a new terminal and try to visualize the robot and its sensors in RViz ``$ rviz rviz``
- Click on the add button, select RobotModel and add it to the RViz panel.
- Visualize the transforms tree: ``$ rosrun rqt_tf_tree rqt_tf_tree``
- Bing up a full robot model with all its transformations ``$ roslaunch turtlebot3_bringup turtlebot3_remote.launch``



## Running turtlebot3 examples
- ``$ roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch``
- ``$ roslaunch turtlebot3_bringup turtlebot3_remote.launch``

Explore the different examples available from the [turtlebot3 metapackage]({https://emanual.robotis.com/docs/en/platform/turtlebot3/basic\_examples/#basic-examples)

### Move using Interactive Markers
- ``$ roslaunch turtlebot3_example interactive_markers.launch``
- ``$ rosrun rviz rviz -d `rospack find turtlebot3_example`/rviz/turtlebot3_interactive.rviz``

![Turtlebot3](https://github.com/HaokunFeng/Robotics_Sensing_Mobility/blob/main/1_Sensors_in_ROS/assets/Figure_5.png)

![rqt_graph](https://github.com/HaokunFeng/Robotics_Sensing_Mobility/blob/main/1_Sensors_in_ROS/assets/Figure_7.png)

### Point Operation
In this example the user specifies an input corresponding to x | y | z. The first two numbers are distance numbers and the third is an angle value ( -180 to 180), referenced to the world origin in Gazebo the robot spawned in the origin (0 | 0 |0). The symbol | represents a space bar character, a correct command for the robot would be “1 1 90”.

- ``$ roslaunch turtlebot3_example turtlebot3_pointop_key.launch``

## Sensors
In this section, we will take a closer look at the information coming from odometry and the LiDAR sensor. We will use the patrol example and create a node which subscribes to the /odom topic to plot the position of the robot throughout the patrol behavior. Additionally, we will create a new node that subscribes to the /scan topic, and moves the robot towards an object that is close by.

### Creating nodes to visualize sensor data from the odometry topic
- Create a node to subscribe to the odometry topic to create an x-y plot of the robot’s reported position.
- Use ``plot_odom.py``
- Install the necessary packages ``$ sudo apt install python-numpy python-matplotlib``

#### Run the patrol example to complete one cycle of 1m square.
![Square](https://github.com/HaokunFeng/Robotics_Sensing_Mobility/blob/main/1_Sensors_in_ROS/assets/Figure_1.png)

#### Select one of the shapes for patrol, and run it with a number of cycles greater than 1 (e.g. t 1 3).
![Triangle](https://github.com/HaokunFeng/Robotics_Sensing_Mobility/blob/main/1_Sensors_in_ROS/assets/Figure_2.png)

![Circle](https://github.com/HaokunFeng/Robotics_Sensing_Mobility/blob/main/1_Sensors_in_ROS/assets/Figure_3.png)