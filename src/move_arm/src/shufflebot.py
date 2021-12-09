import numpy as np
import scipy.spatial
from moveit_group import MoveGroupCommander # TODO: might be wrong import I don't really remember

class ShuffleBot:
    table_z = -0.2
    home_pose = np.array([0, 0, 0, 0, 0, 0, 0])
    puck_poses = np.array([[0, 0, 0, 0, 0, 0, 0], # position/orientation xyzxyzw
                           [0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0]])
    pregrasp = np.mean(puck_poses, axis=0) + np.array([0, 0, 0.1, 0, 0, 0, 0]) # np.array([0, 0, 0, 0, 0, 0, 0])
    num_pucks = puck_poses.shape[0]

    def __init__():
        self.puck = 0
        group = MoveGroupCommander("right_arm")

    def throw(x_pos, y_pos, x_vel, y_vel):
        theta = np.atan2(y_vel, x_vel) - np.pi / 2 # heading from North
        # move robot to grasp puck
        # move to pre-grasp pose


    def move_to_joint_angles(joint_angles):
        # TODO: make this way less awful by using what gtja.py uses rather than os hacking it
        os.system("rosrun intera_examples go_to_joint_angles.py " + str(joint_angles)[1:-2])

    def move_cartesian(displacement_vector, speed):
        R_robot_sim = np.array([[0, 0, 0],
                                [0, 0, 0],
                                [0, 0, 0]])
        robot_disp = np.dot(R_robot_sim, displacement_vector)
        dist = np.linalg.norm(robot_disp)
        t = dist / speed # currently terrified of integer division

        V = robot_disp.append(np.zeros(3))
        # TODO: remove os hacking
        os.system("rosrun move_arm get_manipulator_jacobian.py " + str(V)[1:-2])

    def move_task_space(pose):
        # TODO: i don't remember the message type
        
