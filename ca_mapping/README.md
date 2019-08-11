rosrun stage_ros stageros `rospack find stage_ros`/world/willow-erratic.world

rosrun teleop_twist_keyboard teleop_twist_keyboard.py

rosrun gmapping slam_gmapping scan:=/robot_0/base_scan _odom_frame:=/robot_0/odom _base_frame:=/robot_0/base_link
