# shufflebot

For more information see https://sites.google.com/berkeley.edu/shufflebot

./intera.sh alan
rosrun intera_interface joint_trajectory_action_server.py
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
rosrun intera_examples go_to_joint_angles.py -q 0.0 -0.5 3.14 -1.8 -3.0 0.1 1.57


In order to run this code you need:
Python 2.7.12
OpenCV version 3.3.1.11 (pip install opencv-python==3.3.1.11)
Numpy version 1.15.4 (conda install numpy=1.15.4)
