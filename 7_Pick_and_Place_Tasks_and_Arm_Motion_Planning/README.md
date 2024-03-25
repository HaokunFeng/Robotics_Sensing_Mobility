Pick and Place tasks and Arm Motion Planning
===

## Creating a robotic arm trajectory from waypoints
### Trajectory planning with Bezier curve in Cartesian space for Kinova arm
There are many trajectory planning strategies among which Bezier curve is one that is easy to apply and ideal for applications requiring a smooth trajectory. A Bezier curve is a mathematical curve defined by a series of control points, which determine the shape of the curve. The curve starts at the first control point and ends at the last control point, but it can bend and twist in any way that is defined by the other control points.


<center><img src="https://raw.githubusercontent.com/HaokunFeng/Robotics_Sensing_Mobility/main/7_Pick_and_Place_Tasks_and_Arm_Motion_Planning/assets/bezier2.png" alt=""></center>
<p align="center">Cubic Bezier curve with four contro</p>

- Guide the Kinova arm to follow a trajectory based on a Bezier
curve, moving it from the Home position to the Vertical position.
- Using calculate interpolate points along the Bezier curve based on 4 input control points, with two possible output versions containing 11 or 21 points.