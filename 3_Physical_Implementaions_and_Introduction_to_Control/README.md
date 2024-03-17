# Physical Implementations and Introduction to Control

## Sensor inaccuracies in the physical robot
### Connecting to the physical robot
Connecting to the physical robot according to robot instructions

### Errors in odometry in real world operation
Use the patrol example to compare the odometry performance between simulation and real world implementation.

- Run the patrol example with the following parameters: square length 0.8m and 1 iteration. In an additional terminal run the plot\_odom.py
    ```
    $ roslaunch turtlebot3_example turtlebot3_client.launch
    $ rosrun turtlebot3_example turtlebot3_server
    ```
- Plot the x-y position of the robot against the “ground truth”.
![Position](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/3_Physical_Implementaions_and_Introduction_to_Control/assets/Figure_1.png)

### Errors in laser readings in real world operation
Go through the mapping procedure followed in 2_Mapping_and_Location, and store the /scan topic data into a rosbag. Compare the gmapping results in the real world against the ones obtained in the simulation.

- Make sure the physical turtlebot is operational and placed correctly, in the environment you want to map. Move the robot to the right bottom corner of the map.
![Map](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/3_Physical_Implementaions_and_Introduction_to_Control/assets/map.jpeg)

- Launch the mapping and run the teleoperation program to go through the map.
    ```
    $ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
    $ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
    ```
- Before teleoperating the robot, record a rosbag file with the following topics.
    ```
    $ rosbag record /odom /scan /cmd_vel /tf -O physicaltb3_map.bag
    ```
- When finished tracing the surroundings to complete the map, stop the bag file recording and save the resulting map by running the following command in the terminal before finishing the gmapping launch.
    ```
    $ rosrun map_server map_saver -f ~/gix_map
    ```

![Map3](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/3_Physical_Implementaions_and_Introduction_to_Control/assets/map3.png)