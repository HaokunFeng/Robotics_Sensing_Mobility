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