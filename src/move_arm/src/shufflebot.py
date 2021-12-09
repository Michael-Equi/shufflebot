import numpy as np
import os
import scipy.spatial
import rospy
import intera_interface
from moveit_commander import MoveGroupCommander
import yaml
from intera_interface import gripper as robot_gripper
import time

class ShuffleBot:
    table_z = -0.23
    num_pucks = 4
    numbers = ["one", "two", "three", "four"]
    hit_positions = np.array([0.17, 0.25, 0.36])

    def __init__(self):
        self.puck = 0
        rospy.init_node('shufflebot')
        self.group = MoveGroupCommander("right_arm")
        self.right_arm = intera_interface.limb.Limb("right")
        self.gripper = robot_gripper.Gripper("right")
        f = open("src/poses.yaml", "r")
        self.pose_dict = yaml.load(f)
        self.kick_radius = 0.22
        print(self.pose_dict.keys())

    def poses(self):
        return self.pose_dict.keys()

    def perform_shot(self, x_pos, y_pos, x_vel, y_vel):
        theta = np.arctan2(y_vel, x_vel) - np.pi / 2 # heading from North
        v = np.sqrt(x_vel**2 + y_vel**2)
        hit_vel = v / np.cos(theta)
        ang_vel = hit_vel / self.kick_radius
        x_id = np.argmin(np.abs(ShuffleBot.hit_positions - x_pos)) + 1

        # move robot to home pose
        self.move_to_pose("home")
        raw_input()

        # move to pre-grasp pose
        self.move_to_pose(ShuffleBot.numbers[self.puck])
        time.sleep(2)

        # open gripper
        self.open_gripper()
        raw_input()

        # move to grasp pose
        grasp_pose = self.group.get_current_pose()
        grasp_pose.pose.position.z = ShuffleBot.table_z
        self.move_cartesian([grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z, 0, 0, 0])
        time.sleep(2)

        # close gripper
        self.close_gripper()
        raw_input()

        # return to pre-grasp
        self.move_to_pose(ShuffleBot.numbers[self.puck])
        time.sleep(2)

        # return to home pose
        self.move_to_pose("home")
        raw_input()

        # go to hit pose
        self.move_to_pose("hit" + str(x_id))
        time.sleep(2)

        # open gripper
        self.open_gripper()
        time.sleep(1)

        # raise gripper
        raised_pose = self.group.get_current_pose()
        raised_pose.pose.position.z += 0.04
        self.move_cartesian([raised_pose.pose.position.x, raised_pose.pose.position.y, raised_pose.pose.position.z, 0, 0, 0])
        raw_input()

        # close gripper
        self.close_gripper()
        time.sleep(1)

        # wind up for strike
        j_angles = np.array(self.group.get_current_joint_values())
        j_angles += np.array([0, 0, 0, 0, 0, -1.0, -theta]) # TODO: might be negative
        self.move_to_joint_angles(j_angles)
        raw_input()

        # lower gripper
        lowered_pose = self.group.get_current_pose()
        lowered_pose.pose.position.z -= 0.04
        self.move_cartesian([lowered_pose.pose.position.x, lowered_pose.pose.position.y, lowered_pose.pose.position.z, 0, 0, 0])
        raw_input()

        # strike puck
        vel = lambda i, vi: dict(zip(self.right_arm.joint_names(), [0]*(i) + [vi] + [0]*(6-i)))

        t = 1
        for i in range(int(t * 10)):
            self.right_arm.set_joint_velocities(vel(5, 5*ang_vel)) # TODO: may be negative
            time.sleep(0.1)
        self.right_arm.set_joint_velocities(vel(5, 0))
        raw_input()

        # go to home pose
        self.move_to_pose("home")

        # iterate puck number
        self.puck += 1

    def open_gripper(self):
        self.gripper.open()

    def close_gripper(self):
        self.gripper.close()

    def move_to_pose(self, pose_name):
        pose = self.pose_dict[pose_name]

        js = np.array([float(v) for v in pose['joints'].replace('[','').replace(']', '').replace(' ', '').split(',')])
        self.move_to_joint_angles(js)

    def move_to_joint_angles(self, joint_angles):
        os.system("rosrun intera_examples go_to_joint_angles.py -q " + str(joint_angles)[1:-2].replace('  ', ' ').replace('\n', ''))

    def move_cartesian(self, target_pose):
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
