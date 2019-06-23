#!/usr/bin/env python

###############################################################################
# speed.py
#
# Script to run the speed challenge at the 2019 RoboBoat contest
#
# NOTE: Any plotting is set up for output, not viewing on screen.
#       So, it will likely be ugly on screen. The saved PDFs should look
#       better.
#
# Created: 06/22/19
#   - Joshua Vaughan
#   - joshua.vaughan@louisiana.edu
#   - http://www.ucs.louisiana.edu/~jev9637
#
# Modified:
#   * 
#
# TODO:
#   * 
###############################################################################

import rospy
import actionlib
import numpy as np

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *

mode_pub = rospy.Publisher('/mode', String, queue_size=1, latch=True)

waypoints = [  
    [(15, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(30, 0, 0.0), (0.0, 0.0, 0, 1)],
    [(0, 10, 0), (0, 0 ,0, 1)],
    [(0, 0, 0), (0, 0, 0, -1)],
    [(15, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    [(30, 0, 0.0), (0.0, 0.0, 0, 1)]
]


def goal_pose(pose):  
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'base_link'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose


if __name__ == '__main__':
    rospy.init_node('auto_speedtest')
    
    mode_pub = rospy.Publisher('/mode', String, queue_size=1, latch=True)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  
    client.wait_for_server()

    try:
        # Publish the AUTONOMOUS message since we are going into autonomous mode
        mode_pub.publish('AUTONOMOUS')
        rospy.loginfo('Going autonomous.')
        
        for pose in waypoints:   
            goal = goal_pose(pose)
            
            rospy.loginfo('Sending goal {}'.format(pose))
            client.send_goal(goal)

            # Check the current state
            # It should be
            state = client.get_state()
            
            while state != GoalStatus.SUCCEEDED:
                # Wait one second
                rospy.sleep(1.0) 
                
                # I think we should sleep rather than the below. It seems
                # like it would end the pursuit of that goal
                # client.wait_for_result(rospy.Duration(1.0))
            
                # Check the current state
                state = self.move_base.get_state()
                
            
            rospy.loginfo("Goal succeeded!")

                
    except (KeyboardInterrupt, SystemExit):
        # If we get a keyboard interrupt cancel the goal request and go back
        # to REMOTE on the light tower
        rospy.loginfo('Cancelled goals.')
        client.cancel_goal()
        
        rospy.logdebug('Switching back to REMOTE mode.')
        mode_pub.pulish('REMOTE')


