Arm Kinematics in the Physical Robot
===

## Introducing the Kinova robotic arm as a physical platform
### Connecting to the physical Kinova robotic arm
- Open the browser and type the robot’s IP address. This will open the Kinova Web App.
    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/6_Arm_Kinematics_in_the_Physical_Robot/assets/kinova.PNG)
    <p align="center">Kinova Robot Arm</p>

    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/6_Arm_Kinematics_in_the_Physical_Robot/assets/kinova1.png)
    <p align="center">Kinova Web App</p>
- In the Systems -> Monitoring you will see the current status of the robot’s joints: joint angle values, pose of the end effector with respect to the base, velocities and efforts for each joint.
- In the bottom panel there are two buttons: Pose and Angular. Each one of these, changes the position of the robot arm in a different working space: "Pose" changes the position and orientation of the end effector (task space), while "Angular" changes each joint angle value (configuration space).

    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/6_Arm_Kinematics_in_the_Physical_Robot/assets/kinova2.png)
    <p align="center">Work Space</p>

    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/6_Arm_Kinematics_in_the_Physical_Robot/assets/kinova3.png)
    <p align="center">Configuration Space</p>

    > {Note}: Please be mindful as you use the UI’s joysticks and keep speeds low and movements small, particularly when movements lead towards the table. Please pay close attention to the robot mootion and stop the motion using emergency stop if necessary.

- Use the web app to pick up the target object
    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/6_Arm_Kinematics_in_the_Physical_Robot/assets/Figure_1.jpg)


### Spawning a Kinova robot arm using Kortex Driver
Use the ros\_kortex repository to work with the Kinova arm. Execute the following launch file that will bring up the gen3\_lite controllers, MoveIt! Configurations and an RViZ window.

```
~$ roslaunch kortex_driver kortex_driver.launch arm:=gen3_lite ip_address:=<IP OF ROBOT>
```

> {Note:} Once the RViZ window has been opened, and the terminal should show two green messages ( “You can start planning now!” and “The Kortex driver has been initialized correctly!” ). At this point, you can add a MotionPlanning component in RviZ.

![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/6_Arm_Kinematics_in_the_Physical_Robot/assets/kinovaRviz.png)


### Using the Interactive Markers to change the robot’s pose based on the cartesian-space
- Some useful tools:
    - publishes the values of the joints: ``~$ rostopic echo -n 1 /my_gen3_lite/joint_states``
    - outputs the pose of the robot’s end-effector with respect to the base of the robot: ``~$ rosrun tf tf_echo /base_link <END-EFFECTOR LINK>``
    - generate a PDF with the TF tree for the robot: `` ~$ rosrun tf view_frames``
- To change the state of the gripper, you need to change the Planning Group from arm to gripper, and you can select a new position.

- Home
    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/6_Arm_Kinematics_in_the_Physical_Robot/assets/Figure_2.png)
    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/6_Arm_Kinematics_in_the_Physical_Robot/assets/Figure_3.png)

- Vertical
    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/6_Arm_Kinematics_in_the_Physical_Robot/assets/Figure_4.png)
    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/6_Arm_Kinematics_in_the_Physical_Robot/assets/Figure_5.png)


### Using the Joints tab in MotionPlanning to change the robot’s pose on the configuration-space
In the Motion Planning component, find the Joints tab. You will be able to change the values of the different joint angles. Explore the configuration space, determine which joints move in which direction when a positive or a negative angle is given to each joint.

### Pick up the cube

[![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/6_Arm_Kinematics_in_the_Physical_Robot/assets/Figure_6.png)](https://drive.google.com/file/d/1yum7ISIHVoupH9UqZu44lZcm0edyvq94/view?usp=sharing)


## Intro to Planning Scene Objects in MoveIt! (Only simulation)

Go through the steps of adding an object to our planning scene, which in turn will serve as a constraint when MotionPlanning is creating safe motions for the robot to reach different positions. Using the RViZ tab called Scene Objects, select a “Box” with dimensions 1m for all three axes (x, y, z).

Change the position values to the ones on the picture below (x= 0.55, y = ±0.45, z = -0.48) to set the scene object a little above the table level in the physical world. 
Tick the box next to Box\_0 name (you may rename it), to attach the object to the base\_link.


## Pick and Place task

`` $ roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3_lite``

In Gazebo, on the left panel’s “Insert” tab, find the “Wooden cube 7.5cm” object and add it to the world on a position on the ground within the robot arm’s workspace (space that can be reached by the arm).

Add another planned scene object, to avoid hitting the ground with the robot’s end effector. You can follow the same instructions from RViZ.

The pick and place task can be summarized in the following steps:

    1. Reach position above the center of the object to be picked with gripper oriented
    2. Adjust gripper to open or semi-open state.
    3. Decrease the end-effector’s z-value position
    4. Close gripper to grasp object.
    5. Reach position above the desired location to place the object with gripper oriented.
    6. Decrease the end-effector’s z-value position
    7. Open gripper to release object.
    8. Increase the end-effector’s z-value position to clear the object
    9. Return to home position.

- configure the program by running: ``$ rosrun rqt_reconfigure rqt_reconfigure``
- Set allowed\_execution\_duration\_scaling to be 4 and uncheck the execution\_duration\_monitoring.
    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/6_Arm_Kinematics_in_the_Physical_Robot/assets/config.png)

    [![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/6_Arm_Kinematics_in_the_Physical_Robot/assets/Figure_7.png)](https://drive.google.com/file/d/1uLVzvj29bj41Ow5pKU2XDE0XBUfCFC4S/view?usp=sharing)