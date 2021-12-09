#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""
import sys

from intera_interface import Limb

import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint, JointConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
try:
    from controller import Controller
except ImportError:
    pass
    
def main():
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")

    ##
    ## Add the obstacle to the planning scene here
    ##

    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;

    '''
    jc = JointConstraint()
    jc.joint_name = "right_s5"
    jc.position = 0.1
    jc.tolerance_above = 0.02
    jc.tolerance_below = -0.02
    jc.weight = 1.0
    '''

    # rosrun intera_examples go_to_joint_angles.py -q 0.0 -0.5 3.14 -1.8 -3.0 0.1 1.57

    while not rospy.is_shutdown():

        while not rospy.is_shutdown():
            try:
                x, y, z = 0.8, 0.05, 0.27
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"

                #x, y, and z position
                goal_1.pose.position.x = x
                goal_1.pose.position.y = y
                goal_1.pose.position.z = z

                #Orientation as a quaternion
                goal_1.pose.orientation.x = np.sqrt(0.5)
                goal_1.pose.orientation.y = -np.sqrt(0.5)
                goal_1.pose.orientation.z = 0.0
                goal_1.pose.orientation.w = 0.0

                # Might have to edit this . . . 
                # plan = planner.plan_to_pose(goal_1, {}) # {'joint_constraints':[jc]})
                plan = planner.plan_to_joint_config([0.0, -0.5, 3.14, -1.8, -3.0, 0.1, 1.57])

                raw_input("Press <Enter> to move the right arm to goal pose 1: ")
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break

        while not rospy.is_shutdown():
            try:
                goal_2 = PoseStamped()
                goal_2.header.frame_id = "base"

                #x, y, and z position
                goal_2.pose.position.x = 0.6
                goal_2.pose.position.y = -0.3
                goal_2.pose.position.z = 0.0

                #Orientation as a quaternion
                goal_2.pose.orientation.x = 0.0
                goal_2.pose.orientation.y = -1.0
                goal_2.pose.orientation.z = 0.0
                goal_2.pose.orientation.w = 0.0

                plan = planner.plan_to_pose(goal_2, {}) # {'joint_constraints':[jc]})

                raw_input("Press <Enter> to move the right arm to goal pose 2: ")
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

        while not rospy.is_shutdown():
            try:
                goal_3 = PoseStamped()
                goal_3.header.frame_id = "base"

                #x, y, and z position
                goal_3.pose.position.x = 0.6
                goal_3.pose.position.y = -0.1
                goal_3.pose.position.z = 0.1

                #Orientation as a quaternion
                goal_3.pose.orientation.x = 0.0
                goal_3.pose.orientation.y = -1.0
                goal_3.pose.orientation.z = 0.0
                goal_3.pose.orientation.w = 0.0

                plan = planner.plan_to_pose(goal_3, {}) # {'joint_constraints':[jc]})

                raw_input("Press <Enter> to move the right arm to goal pose 3: ")
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
