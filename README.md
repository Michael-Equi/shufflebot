# shufflebot

./intera.sh alan
rosrun intera_interface joint_trajectory_action_server.py
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
rosrun intera_examples go_to_joint_angles.py -q 0.0 -0.5 3.14 -1.8 -3.0 0.1 1.57
