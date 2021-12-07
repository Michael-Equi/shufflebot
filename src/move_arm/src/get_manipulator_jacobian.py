#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from baxter_interface import gripper as robot_gripper
import time
import intera_interface
import scipy.spatial
import matplotlib.pyplot as plt

def main(vs):
    rospy.init_node('get_jacobian')
    arm = 'right'
    group = MoveGroupCommander(arm + "_arm")
    right_arm = intera_interface.limb.Limb("right")
    time.sleep(1)

    p = group.get_current_pose()
    tra = [p.pose.position.x, p.pose.position.y, p.pose.position.z]
    q = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w]
    rot = scipy.spatial.transform.Rotation.from_quat(q).as_dcm()

    g = np.eye(4)
    g[:3, :3] = rot
    g[:3, 3] = tra

    p_hat = np.array([[0, -tra[2], tra[1]],
                      [tra[2], 0, -tra[0]],
                      [-tra[1], tra[0], 0]])

    adjg = np.eye(6)
    adjg[:3, :3] = rot
    adjg[3:, 3:] = rot
    adjg[:3, 3:] = np.dot(p_hat, rot)

    # Setting position and orientation target
    ss = []
    for i in range(100):
        # print(group.get_current_joint_values())
        js = group.get_current_joint_values()
        Js = group.get_jacobian_matrix(js)

        _, s, _= np.linalg.svd(Js)
        ss.append(np.prod(s))

        # Jb = np.dot(np.linalg.pinv(adjg), Js)

        Jinv = np.linalg.pinv(Js)
        jv = np.dot(Jinv, np.array(vs, dtype=np.float64))
        right_arm.set_joint_velocities(dict(zip(right_arm.joint_names(), jv)))
        # time.sleep(0.001)
    plt.plot(ss)
    plt.show()


# Python's syntax for a main() method
if __name__ == '__main__':
    main(sys.argv[1:])

