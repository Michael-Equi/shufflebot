import rospy
from sensor_msgs.msg import JointState

pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
rospy.init_node('repub')

def cb(js):
    pub.publish(js)

sub = rospy.Subscriber("/robot/joint_states", JointState, cb)

rospy.spin()
