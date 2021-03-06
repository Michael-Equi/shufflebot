import numpy as np
import scipy.spatial
import rospy
import intera_interface
from moveit_commander import MoveGroupCommander # TODO: might be wrong import I don't really remember

class ShuffleBot:
    table_z = -0.2
    home_pose = np.array([0, 0, 0, 0, 0, 0, 0])
    puck_poses = np.array([[0, 0, 0, 0, 0, 0, 0], # position/orientation xyzxyzw
                           [0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0]])
    pregrasp = np.mean(puck_poses, axis=0) + np.array([0, 0, 0.1, 0, 0, 0, 0]) # np.array([0, 0, 0, 0, 0, 0, 0])
    num_pucks = puck_poses.shape[0]

    def __init__(self):
        self.puck = 0
        rospy.init_node('shufflebot')
        self.group = MoveGroupCommander("right_arm")
        self.right_arm = intera_interface.limb.Limb("right")


    def throw(x_pos, y_pos, x_vel, y_vel):
        theta = np.atan2(y_vel, x_vel) - np.pi / 2 # heading from North
        # move robot to grasp puck
        # move to pre-grasp pose


    def move_to_joint_angles(joint_angles):
        # TODO: make this way less awful by using what gtja.py uses rather than os hacking it
        os.system("rosrun intera_examples go_to_joint_angles.py " + str(joint_angles)[1:-2])

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

    def move_task_space(pose):
        # TODO: i don't remember the message type
        pass
        

if __name__ == "__main__":
    sb = ShuffleBot()
    def to_vector(pose):
            quat = np.array([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            angles = scipy.spatial.transform.Rotation.from_quat(quat).as_euler('xyz')
            return np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 0, 0, 0])

    f = open('poses.yaml', 'a')
    while True:
        name = raw_input("Type name and press enter to save pose: ")
        pose = to_vector(sb.group.get_current_pose())
        js = sb.group.get_current_joint_values()
        formatted = name + ":\n    pose: \"" + str(list(pose)) + "\"\n    joints: \"" + str(list(js)) + "\"\n"
        f.write(formatted)
        print("Pose saved")
