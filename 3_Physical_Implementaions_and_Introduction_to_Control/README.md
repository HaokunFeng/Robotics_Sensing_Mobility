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
