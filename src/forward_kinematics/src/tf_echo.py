#!/usr/bin/env python
import tf2_ros
import sys
import rospy

if __name__ == "__main__":
	rospy.init_node("tf_echo", anonymous=True)
	target_frame = sys.argv[1]
	source_frame = sys.argv[2]


	tfBuffer = tf2_ros.Buffer()
	tfListener = tf2_ros.TransformListener(tfBuffer)

	while not rospy.is_shutdown():
		try:
			tra = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
			print(tra)
			rospy.sleep(1)
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			print(e)