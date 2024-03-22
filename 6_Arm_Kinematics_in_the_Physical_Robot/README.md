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
    