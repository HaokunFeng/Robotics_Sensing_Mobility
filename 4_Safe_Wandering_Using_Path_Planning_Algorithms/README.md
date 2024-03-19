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



