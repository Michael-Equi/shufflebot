#!/usr/bin/env python
import rospy
import sys

if sys.argv[1] == 'sawyer':
	from intera_interface import gripper as robot_gripper
else:
    from baxter_interface import gripper as robot_gripper

rospy.init_node('gripper_close')

# Set up the right gripper
right_gripper = robot_gripper.Gripper('right')

# Close the right gripper
print('Closing...')
right_gripper.close()
rospy.sleep(1.0)