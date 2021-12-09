import numpy as np
import os
import scipy.spatial
import rospy
import intera_interface
from moveit_commander import MoveGroupCommander # TODO: might be wrong import I don't really remember
import yaml
from intera_interface import gripper as robot_gripper
import time

class ShuffleBot:
    table_z = -0.2
    num_pucks = 4
    numbers = ["one", "two", "three", "four"]

    def __init__(self):
        self.puck = 0
        rospy.init_node('shufflebot')
        self.group = MoveGroupCommander("right_arm")
        self.right_arm = intera_interface.limb.Limb("right")
        self.gripper = robot_gripper.Gripper("right")
        f = open("poses.yaml", "r")
        self.pose_dict = yaml.load(f)

    def poses(self):
        return self.pose_dict.keys()

    def throw(x_pos, y_pos, x_vel, y_vel):
        theta = np.atan2(y_vel, x_vel) - np.pi / 2 # heading from North
        # move robot to home pose
        self.move_to_pose("home")
        raw_input()

        # move to pre-grasp pose
        self.move_to_pose(numbers[self.puck])
        time.sleep(1)

        # open gripper
        self.open_gripper()
        raw_input()

        # move to grasp pose
        to_pose = self.group.get_current_pose()
        

        # iterate puck number
        self.puck += 1

    def open_gripper(self):
        self.gripper.open()

    def close_gripper(self):
        self.gripper.close()

    def move_to_pose(self, pose_name):
        try:
            pose = pose_dict[name]
        except:
            raise Exception("invalid pose")
            continue
        js = np.array([float(v) for v in pose['joints'].replace('[','').replace(']', '').replace(' ', '').split(',')])
        self.move_to_joint_angles(js)

    def move_to_joint_angles(self, joint_angles):
        os.system("rosrun intera_examples go_to_joint_angles.py -q " + str(joint_angles)[1:-2].replace('  ', ' ').replace('\n', ''))

    def move_cartesian(self, target_pose, speed):
        Kp = np.diag(np.array([1, 1, 1, 0.1, 0.1, 0.1]))
        Kd = 0
        Ki = 0


        def to_vector(pose):
            quat = np.array([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            angles = scipy.spatial.transform.Rotation.from_quat(quat).as_euler('xyz')
            return np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 0, 0, 0])

        pose = to_vector(self.group.get_current_pose())
        print(pose)        
        while np.linalg.norm(pose - target_pose) > 0.001 and not rospy.is_shutdown():

            # Target pose is in the frame of the robot base
            js = self.group.get_current_joint_values()
            pose = to_vector(self.group.get_current_pose())
            Js = self.group.get_jacobian_matrix(js)
            Jinv = np.linalg.pinv(Js)

            e = target_pose - pose
            print(np.linalg.norm(e))
            vel = np.dot(Kp, e)

            jv = np.dot(Jinv, np.array(vel, dtype=np.float64))
            _, s, _= np.linalg.svd(Js)
            print "manipulability: " +  str(np.prod(s))
            self.right_arm.set_joint_velocities(dict(zip(self.right_arm.joint_names(), jv)))

        self.right_arm.set_joint_velocities(dict(zip(self.right_arm.joint_names(), np.array([0,0,0,0,0,0,0]))))



        # robot_disp = np.dot(R_robot_sim, displacement_vector)
        # # dist = np.linalg.norm(robot_disp)
        # t = dist / speed # currently terrified of integer division

        # V = robot_disp.append(np.zeros(3))
        # TODO: remove os hacking
        # os.system("rosrun move_arm get_manipulator_jacobian.py " + str(V)[1:-2])

if __name__ == "__main__":
    sb = ShuffleBot()
    def to_vector(pose):
            quat = np.array([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            angles = scipy.spatial.transform.Rotation.from_quat(quat).as_euler('xyz')
            return np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 0, 0, 0])

    f = open("poses.yaml", "r")
    pose_dict = yaml.load(f)
    print "Poses:", pose_dict.keys()
    while True:
        name = raw_input("Type name and press enter to go to pose: ")
        try:
            pose = pose_dict[name]
        except:
            continue
        js = np.array([float(v) for v in pose['joints'].replace('[','').replace(']', '').replace(' ', '').split(',')])
        sb.move_to_joint_angles(js)
