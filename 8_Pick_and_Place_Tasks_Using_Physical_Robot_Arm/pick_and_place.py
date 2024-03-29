#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run : 
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
import copy
import os

class PickAndPlace(object):
  """PickAndPlace"""
  def __init__(self):
    # Initialize the node
    super(PickAndPlace, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        self.gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True


  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    (success_flag, planned_path1, planning_time, error_code)= arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(planned_path1, wait=True)

  def reach_joint_angles(self, config_pose, tolerance):
    arm_group = self.arm_group
    success = True

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    arm_group.set_joint_value_target(config_pose)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def plan_cartesian_path(self, waypoints, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.arm_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.arm_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 

def main():
  example = PickAndPlace()

  # For testing purposes
  success = example.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass

  if success:

    # rospy.loginfo("Added cartesian path functions")
    
    
    success &= example.reach_named_position("retract")
    
    rospy.loginfo("reaching retract position")
    
    success &= example.reach_gripper_position(1)
    
    joint_pose_above_object = [-0.8311608583049344, -0.4359381651572507, 1.7130011395218576, -1.5836450137900924, -1.002298785746734, -0.8779962744995036]
    
    
    
    success &= example.reach_joint_angles(config_pose = joint_pose_above_object,tolerance=0.01) #rad
    rospy.loginfo("above object position")
    
    joint_pose_object = [-0.8048051508930731, -0.7862738107638663, 1.7956400934114523, -1.5909314225325497, -0.5712256832688407, -0.8412963167815573]
    
    
    
    success &= example.reach_joint_angles(config_pose = joint_pose_object,tolerance=0.01) #rad
    rospy.loginfo("reach the object")
    
    success &= example.reach_gripper_position(0.6)
    
    rospy.loginfo("prepare to grip")
    
    success &= example.reach_gripper_position(0.35)
    
    rospy.loginfo("gripped")

    pos1 = [-0.7778614175125238, -0.5593628985091907, 0.24793550379641494, -1.534790388857025, -2.378816712072023, -0.7968066128751294]
    
    success &= example.reach_joint_angles(config_pose = pos1, tolerance=0.01) #rad
    
    rospy.loginfo("lifting up")
    
    pos2 = [0.11634764907116374, -1.4561525681891654, 0.8029409381300182, 0.06094937102366245, 1.6510634694176667, 0.6852609095654879]
    
    success &= example.reach_joint_angles(config_pose = pos2, tolerance=0.01) #rad
    
    rospy.loginfo("above target position")
    
    pos3 = [0.1162088983783706, -1.7094213183845728, 0.5467315254589451, 0.060032178344239163, 1.6517106175625562, 0.6819466055888843]
    
    success &= example.reach_joint_angles(config_pose = pos3, tolerance=0.01) #rad
    
    rospy.loginfo("prepare to put down")

    success &= example.reach_gripper_position(0.6)
    
    rospy.loginfo("releasing")
    
    pos4 = [0.1297089945762003, -0.7190518964247401, 0.7772113398904357, -0.011966115409943967, 1.685048600720386, -0.0820253615744404]
    
    success &= example.reach_joint_angles(config_pose = pos4, tolerance=0.01) #rad
    
    rospy.loginfo("above target position")
    
    success &= example.reach_named_position("retract")
    
    return 

    # Joint configuration found from RViz IN RADIANS


    rospy.loginfo("Making sure gripper is open")
    
    success &= example.reach_gripper_position(0)

    rospy.loginfo("Lowering in z-axis")
    
    actual_pose = example.get_cartesian_pose()
    actual_pose.position.z -= 0.06
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
     
    rospy.loginfo("Closing the gripper 30%...")
    
    success &= example.reach_gripper_position(0.3)

    rospy.loginfo("Ascending in z-axis")
    
    actual_pose = example.get_cartesian_pose()
    actual_pose.position.z += 0.15
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
     
    rospy.loginfo("Reaching Pose above green bin")

    # Joint configuration found from RViz
    joint_pose_above_green_bin = [-0.8705235572878225, 0.7132756691134601, 3.154655446693461, -1.0576463485761654, -0.4489708882993586, -0.9799159153742165, 2.4297948935169877]
    
    success &= example.reach_joint_angles(config_pose = joint_pose_above_green_bin,tolerance=0.01) #rad
    
    rospy.loginfo("Intermediate pose between bins")
    
    actual_pose = example.get_cartesian_pose()
    actual_pose.position.x = -0.171
    actual_pose.position.y = 0.417
    actual_pose.position.z = 0.846
    actual_pose.orientation.x = -0.091
    actual_pose.orientation.y = 0.763
    actual_pose.orientation.z = 0.63 
    actual_pose.orientation.w = 0.115
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
     
    rospy.loginfo("Reaching Pose above red bin")

    # Joint configuration found from RViz
    joint_pose_above_red_bin = [1.9494411746376308, 0.49489731216768185, 4.132505629522368, -1.4148809908124065, -0.779860266487816, -1.2977438326879174, -0.0063647021418793415]
    
    success &= example.reach_joint_angles(config_pose = joint_pose_above_red_bin,tolerance=0.01) #rad
    
    rospy.loginfo("Making sure gripper is open")
    
    example.reach_gripper_position(0)

    rospy.loginfo("Finishing by reaching Vertical")
    
    success &= example.reach_named_position("vertical")

    # For testing purposes
    rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

    if not success:
        rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
  main()
