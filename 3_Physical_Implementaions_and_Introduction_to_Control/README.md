Physical Implementations and Introduction to Control
===

# Sensor inaccuracies in the physical robot
## Connecting to the physical robot
Connecting to the physical robot according to robot instructions

## Errors in odometry in real world operation
Use the patrol example to compare the odometry performance between simulation and real world implementation.

- Run the patrol example with the following parameters: square length 0.8m and 1 iteration. In an additional terminal run the plot\_odom.py
    ```
    $ roslaunch turtlebot3_example turtlebot3_client.launch
    $ rosrun turtlebot3_example turtlebot3_server
    ```
- Plot the x-y position of the robot against the “ground truth”.
![Position](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/3_Physical_Implementaions_and_Introduction_to_Control/assets/Figure_1.png)

## Errors in laser readings in real world operation
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

    ![Map2](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/3_Physical_Implementaions_and_Introduction_to_Control/assets/Figure_2.png)

- Use that map created and launch the navigation example.
    ```
    $ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/gix_map.yaml
    ```
- Set a 2DNavGoal on the map.
    ![Map](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/3_Physical_Implementaions_and_Introduction_to_Control/assets/map2.jpeg)



# Introduction to control strategies in robot navigation
Explore different paradigms to control the movements of a robotic platform

## Executing robot motions in open-loop control
- Launch a simulated turtlebot3 in the stage\_1 world: ``$ roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch``
- Determine the constant speed and the amount of time that constant speed needs to be held to cover said distance. Use the command-tool rostopic to publish velocity commands to the robot: ``$ rostopic pub -1 /cmd_vel geometry_msgs/Twist  '{linear:  {x: <value>, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'``
- After a certain amount of time (pos = vel * time), publish a twist message to effectively stop the robot from moving: ``$ rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear:{x: 0, y: 0.0, z: 0.0}, angular:{x: 0.0,y: 0.0,z: 0.0}}'``
- How would the sequence of commands look if you wanted to complete a patrol pattern (e.g. a triangle) in open-loop?
    ```
    #pseudocode
    Initialize Robot
    # Define patrol pattern parameters
    side_length = 1
    rotation_angle = 120.0
    movement_speed = 0.5

    # Patrol pattern commands
    GoForward(side_length)
    WaitTime(3.0)

    Rotate(rotation_angle)
    WaitTime(3.0)

    GoForward(side_length)
    WaitTime(3.0)

    Rotate(rotation_angle)
    WaitTime(3.0)

    GoForward(side_length)
    WaitTime(3.0)

    Shutdown Robot
    ```
- Modify the given open loop file to achieve this patrol motion.
    ```
    import os
    import time

    def GoForward(distance, speed):
        os.system(f"rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[{speed}, 0.0, 0.0]' '[0.0, 0.0, 0.0]'")
        time.sleep(distance / speed)

    def Rotate(angle, speed):
        os.system(f"rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, {speed}]'")
        time.sleep(angle / speed)

    # Define patrol pattern parameters
    side_length = 0.6  # Length of each side of the triangle
    rotation_angle = 1.2  # Angle to turn at each corner for a triangle
    movement_speed = 0.2  # Speed at which the robot moves forward

    # Perform triangular motion
    for _ in range(3):  # Repeat the pattern three times for a triangle
        GoForward(side_length, movement_speed)  # Move forward for the length of one side
        Rotate(rotation_angle, movement_speed)  # Turn by the specified angle

    # Stop the robot
    os.system("rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'")
    ```

## Executing robot motions in closed-loop control
![close_loop_odom.py](https://drive.google.com/file/d/1BLsGuUQjAVZ0VQYrjQjnLBMr8a240UXC/view?usp=sharing)

![close_loop_laser.py](https://drive.google.com/file/d/1nCw97bPlgRkBqKBgahunCjbnzLAQxz7U/view?usp=sharing)
