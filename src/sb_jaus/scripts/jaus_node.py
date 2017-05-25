#! /usr/bin/env python

# Created By: Gareth Ellis
# Created On: May 25, 2017
# TODO: Get Finn to elaborate on below description
# Description: This node acts as the interface between Snowbot's in-house
#               JAUS library and ROS

import rospy

from sensor_msgs import Odometry

if __name__ = '__main__':
    rospy.init_node('jaus_api')

    rospy.Subscriber('/robot_pose_ekf/odom_combined', Odometry, 
            robot_state_callback)

    rospy.spin()

