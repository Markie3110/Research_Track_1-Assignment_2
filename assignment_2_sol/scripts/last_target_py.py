#! /usr/bin/env python

## @package assignment_2_sol
# \file last_target_py.py
# \brief A rosnode that monitors the last position specified by the user.
# \author Mark Henry Dsouza
# \date 15/05/2024
#
# \details 
#
# Description: <BR>
# Implements the ROS node "last_target" that creates the rosservice "last_target_serv" which when 
# called, prints the latest goal position specified by the user. Obtains the latest goal 
# via a subscription to the "/goal" topic of the "reaching_goal" actionserver.
#
# Subscribes to: <BR>
#   - /reaching_goal/goal
#
# Publishes to: <BR>
#   - [None]
#
# Services implemented: <BR>
#   - last_target_serv
#
# Action Servers called: <BR>
#   - [None]

## The necessary libraries
import rospy
import assignment_2_2023.msg
from assignment_2_sol.srv import *
import geometry_msgs.msg
import sys


## Holds a Point message describing the target given by the user
target_coord = geometry_msgs.msg.Point()

## \brief Callback function for the subscriber to /reaching_goal/goal.
#
# Copies the pose specified within msg to the local variable target_coord holding the Point message.
# 
# \param msg A message (geometry_msg/Point) that contains the latest target sent by the user.
#
# \return None
def target_callback(msg):
    target_coord.x = msg.goal.target_pose.pose.position.x
    target_coord.y = msg.goal.target_pose.pose.position.y
    target_coord.z = msg.goal.target_pose.pose.position.z

## \brief Callback function for last_target_serv service
#
# A callback that is associated with the last_target_serv service server that sends the last known target of the robot when called upon.
# 
# \param req Contains info about the request sent by a caller to the service server. Required for the function declaration.
#
# \return resp Contains the x, y and z values of the last target specified by the user (assignment_2_sol/LastCoord -> Response).
def last_target_callback(req):
    resp = LastCoordResponse()
    resp.last_x = target_coord.x
    resp.last_y = target_coord.y
    resp.last_z = target_coord.z
    return resp


# Run the main
if __name__ == '__main__':
    try:
        rospy.init_node('last_target')
        ## Holds the instance of the subscriber to /reaching_goal/goal
        sub = rospy.Subscriber('/reaching_goal/goal', assignment_2_2023.msg.PlanningActionGoal, target_callback)
        ## Holds the instance of the service 'last_target_serv'
        service_serv = rospy.Service('last_target_serv', LastCoord, last_target_callback)
        rospy.Rate(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Program interrupted:", file = sys.stderr)