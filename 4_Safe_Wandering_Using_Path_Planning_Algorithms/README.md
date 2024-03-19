Safe wandering using path planning algorithms
===

## Install and test the global\_planner package

- Download global\_planner folder.
- Place the global\_planner folder in your src folder in your workspace (~/catkin/src/). You will need to build your package from the /catkin\_ws/ directory. Make sure that all dependencies are installed before building the package:
    ```
    $ rosdep install --from-paths src --ignore-src -r -y
    $ catkin_make global_planner
    ```
- source devel/setup.bash, launch a turtlebot3 gazebo instance and run the test\_planner launch file:
    ```
    $ roslaunch turtlebot3_gazebo turtlebot3_world.launch
    $ roslaunch global_planner test_planner.launch
    ```
- You should be able to see the following topics as active after rostopic list:
    ```
    /move\_base/GlobalPlanner/plan
    /move\_base/GlobalPlanner/potential
    ```
- Once find these topics, the package is successfully installed.


## Path planning algorithms

### Adding the global planner to the turtlebot3 navigation stack

In previous project, turtlebot3\_navigation.launch is launched before we used the 2D Nav Goal. If you look into the launch file, you will notice it launches move\_base.launch -- it is where to configure the planners in turtlebot's navigation stack. We want to create a new navigation stack by duplicating the launch files and modifying the copied files as needed.

- Make copies of launch files in the turtlebot3\_navigation
    > move\_base.launch → move\_base\_path\_planning.launch: the changes made to this new file are related with adding the global\_planner. Details will be provided later.

    >turtlebot3\_navigation.launch → turtlebot3\_path\_planning.launch: the main difference in this new file is launching the move\_base\_path\_planning.launch file created in step a.

    >You need to make the change in the "turtlebot3\_path\_planning.launch" file on the move\_base node to include the new move\_base\_path\_planning.launch file


- Download the parameters .yaml file (global\_planner\_params\_burger.yaml) and place it in the turtlebot3\_navigation/param folder. This file allows you to make changes to the global planner parameters. Refer to the [ROS wiki](http://wiki.ros.org/global\_planner) for details on what each parameter does.

- In the move\_base\_path\_planning.launch file, add a global planner as a parameter of the move\_base node in the launch file:
    ```
    <param name="base\_global\_planner" value="global\_planner/GlobalPlanner" />
    <rosparam file="\$(find turtlebot3\_navigation)/param/global\_planner\_params\_burger.yaml" command="load"/>
    ```
    ![Global Planner yaml file](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/4_Safe_Wandering_Using_Path_Planning_Algorithms/assets/globalPlanner.png)

    <p align="center">Global Planner yaml file</p>

### Run the new navigation stack
- Launch a gazebo instance of the burger in the turtleworld3\_world: ``$ roslaunch turtlebot3_gazebo turtlebot3_world.launch``

- Launch the new turtlebot3\_path\_planning.launch. Before sending any Navigation goals, provide a 2DEstimate for the actual position of the robot in RViz. ``$ roslaunch turtlebot3_navigation turtlebot3_path_planning.launch``

- The 2DEstimate point with the RViz GUI provides the robot’s initial position and orientation; a large mismatch between the robot’s estimates and its actual position may result in a fatal error for the launched file.

- Customize the RViz visualization components
    > Turn off the Global Map and Local Map, and add by topic the GlobalPlanner/potential topic as another map. Choose costmap to be the color scheme.

    > Change the topic of Planner Plan to GlobalPlanner/plan.

- Publish four sequential points for the robot to navigate towards($P_0 \rightarrow P_1\rightarrow P_2 \rightarrow P_3 \rightarrow P_0$). publish in the /move\_base/goal topic through a node (path\_planning\_goal.py from Canvas), or use rostopic pub from the command line.

    ![Four Sequential Points](https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/4_Safe_Wandering_Using_Path_Planning_Algorithms/assets/Fourpoints.PNG)
    
    <p align="center">Four Sequential Points</p>

- Use two global planners: Djikstra and A* 
    - When using dijkstra the planner works best if using gradient descent and not a grid; the opposite is true for A*. Also the value for neutral\_cost is relevant (lower for Dijkstra, medium for A*). The option of use\_quadratic may or may not help. The cost\_factor might influence how much the local planner will be used.
    - Some parameters can be changed dynamically using the command: ``$ rosrun rqt_reconfigure rqt_reconfigure``

- Use the pillar model, and add it as an obstacle in the turtlebot3\_world. Turn on the Local Map visualization in Rviz. Attempt to send a 2DNavigationGoal where the robot has to circumvent the new obstacle.
    ![]