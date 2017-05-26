#! /usr/bin/env python

# Created By: Gareth Ellis
# Created On: May 25, 2017
# TODO: Get Finn to elaborate on below description
# Description: This node acts as the interface between Snowbot's in-house
#               JAUS library and ROS

import rospy

from nav_msgs.msg import Odometry

def robot_state_callback(odom_msg, jaus_api):
    print("Linear X: " + str(odom_msg.linear.x))

if __name__ = '__main__':
    rospy.init_node('jaus_api')

    # Here "JausAPI" is a placeholder for whatever you need to pass
    # into the callback
    jaus_api = CreateJausAPI();

    rospy.Subscriber('/robot_pose_ekf/odom_combined', Odometry, 
            robot_state_callback, jaus_api)

    # You should spawn your thread for Jaus stuff here
    # then return so "spin()" can run
    rospy.spin()

