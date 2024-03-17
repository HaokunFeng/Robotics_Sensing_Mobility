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

