#! /usr/bin/env python
###################################
# This file find the goal and publish a relative tf GF_02
#
#
###################################
import rospy
import roslib
import math
import tf
from ca_main.srv import GoalCustomSrv, GoalCustomSrvRequest # you import the service message python classes generated from Empty.srv.
#from std_srvs.srv import Empty, EmptyResponse # you import the service message python classes generated from Empty.srv.
#from geometry_msgs.msg import Twist

## vars
goal_x = 0
goal_y = 0
goal_yaw = 0

def GoalUpdate():
    global goal_x, goal_y, goal_yaw
    rospy.wait_for_service('/get_goal')
    goal_srv_client = rospy.ServiceProxy('/get_goal', GoalCustomSrv)
    goal_srv_client_obj = GoalCustomSrvRequest()
    
    rospy.loginfo("Doing /get_goal Service Call...")
    result = goal_srv_client(goal_srv_client_obj)
    goal_x = result.x
    goal_y = result.y
    goal_yaw = result.t
    rospy.loginfo("END of /get_goal Service call...")
    


if __name__ == '__main__':
    rospy.init_node('main_program_node')
    ## Update Goal/Get Current Goal
    GoalUpdate()
    rospy.loginfo("Goal Recieved: Goal_x: " + str(goal_x) + " Goal_y: " + str(goal_y) + " Yaw: " + str(goal_yaw))


    ## Based on generated goal, add a tf
    rospy.loginfo("Addign respective goal frame w.r.t /robot_0_odom")
    br1 = tf.TransformBroadcaster()
    br2 = tf.TransformBroadcaster()
    #br.sendTransform((3.5, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "global_frame", "/map")

    #br.sendTransform((0.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "global_frame", "/map")

    rate = rospy.Rate(10.0)
    #for 3rd quad
 #   goal_yaw =  goal_yaw - 0.52333 - 0.52333/2
#for 2nd quad
#    goal_yaw =  goal_yaw + 0.52333/2
#for first quad
#    goal_yaw = goal_yaw + 3.14 - 3.14/2 + 3.14/6
#for fourth quad
#    goal_yaw = goal_yaw + 3.14/2 - 0.52333/2
    goal_yaw = goal_yaw + 3.14
 #   br_x = 3.5*math.cos(goal_yaw )
  #  br_y = 3.5*math.sin(goal_yaw )

#For a: 
    #goal_yaw = goal_yaw + 3.14 - 3.14/2 + 3.14/6

#    goal_yaw = goal_yaw + 0.52333/2

    br_x = 3.5*math.cos(goal_yaw )
    br_y = 3.5*math.sin(goal_yaw )
    while not rospy.is_shutdown():
       # rospy.loginfo("br_x: " + str(br_x) + "br_y: " + str(br_y))
        br1.sendTransform((br_x, br_y, 0.0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "map",
            "GF_02")
    
    rospy.spin()
    
#    rate.sleep()
