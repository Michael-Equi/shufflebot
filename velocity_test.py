import intera_interface
import rospy
import time
import sys

rospy.init_node('velocity_test')
right_arm = intera_interface.limb.Limb("right")
time.sleep(1)


v = lambda i, vi: dict(zip(right_arm.joint_names(), [0]*(i) + [vi] + [0]*(6-i)))


print("Sending commands")

t = float(sys.argv[1])
print "Moving for", t, "seconds" 
for i in range(int(t * 10)):
	right_arm.set_joint_velocities(v(3, float(sys.argv[2])))
	time.sleep(0.1)
right_arm.set_joint_velocities(v(5, 0))
print("Done")
