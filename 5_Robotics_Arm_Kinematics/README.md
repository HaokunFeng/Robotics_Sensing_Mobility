Robotics Arm Kinematics
===

## Intro to Kinova Gen3lite
### ROS Kinematics Package Installation
- ROS Kortex is the official ROS package to interact with Kortex and its related products. These are the instructions to run in a terminal to clone the ros\_kortex repository and install the necessary ROS dependencies.
- {Note}:The default branch for git clone is developed for ROS Noetic. If you are using other ROS distributions, please git clone to corresponding branch.
    ```
    $ sudo apt install python3 python3-pip
    $ sudo python3 -m pip install conan==1.59
    $ conan config set general.revisions_enabled=1
    $ conan profile new default --detect > /dev/null
    $ conan profile update settings.compiler.libcxx=libstdc++11 default
    $ cd catkin_ws/src
    $ git clone https://github.com/Kinovarobotics/ros_kortex.git
    $ cd ../
    $ rosdep install --from-paths src --ignore-src -y
    ```
- to build and source the workspace:
    ```
    $ catkin_make
    $ source devel/setup.bash
    ```

### Spawning a Kinova Gen3lite robot in Gazebo
Use the ros\_kortex repository to work with the Kinova arm. In this metapackage, use the kortex\_gazebo portion for simulation. Execute the following launch file that will bring up a gen3lite in Gazebo and open an RViZ window: ``$ roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=gen3_lite z0:=0.8``

- Note: Once the RViZ and Gazebo windows have been opened, and the terminal shows two green messages ( “You can start planning now!” and “The Kortex driver has been initialized correctly!” ) Then add a Robot model and add a MotionPlanning component in Rviz.


### Using the Interactive Markers to change the robot’s pose based on the task-space
Use the different arrows and rings on the interactive marker, to change positions and orientations of the robot’s end-effector. As you use them, you will see an orange version of the robot arm with the proposed new position. Once you have reached a desired position, use the buttons “Plan” and “Execute”(in the MotionPlanning Panel) to make the robot in Gazebo move to the new proposed position. Some useful tools to track the robot’s motions are:
- Check the values of the joints: ``$ rostopic echo -n 1 /my_gen3_lite/joint_states``
- Outputs the pose of the robot’s end-effector with respect to the world (in this case Gazebo’s origin): ``$ rosrun tf tf_echo /world /end_effector_link``
- Use the base link as the reference: ``$ rosrun tf tf_echo /base_link <END-EFFECTOR LINK>``
- Add a TF component in RViz and enable only the base\_link and the end\_effector\_link. Disable the Motion Planning component to temporarily remove the interactive markers.
    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/5_Robotics_Arm_Kinematics/assets/Figure_1.png)
    <p align="center">Kinova in RViz</p>

- Use rqt\_tf\_tree to determine the kinematic chain between the two reference frames (base\_link and end\_effector\_link).
    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/5_Robotics_Arm_Kinematics/assets/Figure_2.png)
    <p align="center">Nodes and Topics</p>

- Turn on the Motion Planning component again. The "Goal State" in the MotionPlanning panel allows you to set goals to some predefined positions. Check the robot’s joints values and pose (wrt the world) when you send the gen3lite to the following preset positions:
    - Home
        ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/5_Robotics_Arm_Kinematics/assets/Figure_3.png)
        ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/5_Robotics_Arm_Kinematics/assets/Figure_4.png)
    - Retract
        ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/5_Robotics_Arm_Kinematics/assets/Figure_5.png)
        ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/5_Robotics_Arm_Kinematics/assets/Figure_6.png)
    -  Select a robot pose that you would choose if you had to pick something from the table
        ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/5_Robotics_Arm_Kinematics/assets/Figure_7.png)
        ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/5_Robotics_Arm_Kinematics/assets/Figure_8.png)

### Using the Joints tab in MotionPlanning to change the robot’s pose on the configuration-space
In the MotionPlanning panel, find the "Joints" tab. You will be able to change the values of the different joint angles.
    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/5_Robotics_Arm_Kinematics/assets/Figure_9.png)
    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/5_Robotics_Arm_Kinematics/assets/Figure_10.png)


## Intro to Fetch Robot
### Fetch Packages Installation
Following the instructions to install packages from from Fetch Robotics.

```
    $ cd catkin_ws/src
    $ git clone -b ros1 https://github.com/fetchrobotics/fetch_ros.git
    $ git clone -b gazebo11 https://github.com/fetchrobotics/fetch_gazebo.git
    $ git clone -b ros1 https://github.com/fetchrobotics/fetch_msgs.git
    $ git clone -b ros1 https://github.com/fetchrobotics/power_msgs.git
    $ git clone -b ros1 https://github.com/fetchrobotics/robot_controllers.git
    $ cd ../
    $ rosdep install --from-paths src --ignore-src -y
    (Ignore the error: Unable to locate package ros-noetic-simple-grasping)
    $ sudo apt install ros-noetic-rgbd-launch
```
> {Note}:The branch "ros1" and "gazebo11" is developed for ROS Noetic. If you are using other ROS distributions, please git clone to corresponding branch.

### Fetch Robot Manipulation
- Execute the following commands in different terminals to spawn a Gazebo simulated Fetch robot, activate the motion capabilities and open an RViZ window
    ```
    $ roslaunch fetch_gazebo simulation.launch
    $ roslaunch fetch_moveit_config move_group.launch
    $ rosrun rviz rviz
    ```
- Add the Robot Model and the MotionPlanning components in RViZ once it opens. Check the rqt\_tf\_tree to determine the name of the end-effector’s link to run the command: ``$ rosrun tf tf_echo /odom <END-EFFECTOR LINK>``
- Use the start of the arm chain as the reference: ``$ rosrun tf tf_echo /<FIRST ARM JOINT> <END-EFFECTOR LINK>``

- Add a TF component in RViz and enable only the base\_link, the first joint of the arm, and the end\_effector\_link. Disable the Motion Planning component to remove temporarily the interactive markers.
    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/5_Robotics_Arm_Kinematics/assets/Figure_11.png)

- Use rqt\_tf\_tree to determine the kinematic chain between the two reference frames.
    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/5_Robotics_Arm_Kinematics/assets/Figure_12.png)

- Unlike the Kinova arm, the MotionPlanning for Fetch does not have preset poses for the arm.
- All joints in the arm are set to 0 degrees.
    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/5_Robotics_Arm_Kinematics/assets/Figure_13.png)

- Joint angle values of all Fetch’s arm joints on a pose that would allow Fetch to pick up an object from the floor.
    ![](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/5_Robotics_Arm_Kinematics/assets/Figure_14.png)


## Using Python to control Fetch robots

- Use simple_disco.py for forward kinematics, and use wave.py for inverse kinematics
    [![]()]()