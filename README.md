# Castaway on Island

### Problem Description

To be updated.

### 101: Steps to setup

First, run the ```roscore```

```source ~/.bashrc```

```source ~/catkin_ws/devel/setup.bash```

To launch Rviz, Stage simulator with world file, and tf_broadcasters:

```roslaunch ca_mapping tf_static_pub.launch```

For map_merger package: 

```roslaunch ca_mapping map_merge.launch```

For AMCL:

```roslaunch ca_loc amcl.launch```

For Path Planning:

```roslaunch ca_path_planning move_base3.launch```


### Other useful commands
```rosrun stage_ros stageros island.world ```

```rosrun tf static_transform_publish 0 0 0 0 0 0 /map /robot_0/odom 1000```

```rosrun rqt_tf_tree rqt_tf_tree```

```rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot_1/cmd_vel```

```rosrun map_server map_saver -f map_l_1 map:=/map_merged_1```

