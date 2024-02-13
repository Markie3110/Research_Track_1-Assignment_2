#! /usr/bin/env python


# Import the necessary libraries
import rospy
import assignment_2_2023.msg
from assignment_2_sol.srv import *
import geometry_msgs.msg
import sys


# Declare a variable to hold the message describing the target given by the user
target_coord = geometry_msgs.msg.Point()


def target_callback(msg):
    """ A callback that handles the new data describing the latest robot target. 

    Args:
        msg (geometry_msg/Point): A message that contains the latest target sent by the user.
    """
    target_coord.x = msg.goal.target_pose.pose.position.x
    target_coord.y = msg.goal.target_pose.pose.position.y
    target_coord.z = msg.goal.target_pose.pose.position.z


def last_target_callback(req):
    """  A callback that is associated with the last_target_serv service server that sends the last known target of the robot when called upon.

    Args:
        req (): Contains info about the request sent by a caller to the service server. Required for the function declaration.
        
    Returns:
        resp (assignment_2_sol/LastCoord -> Response): Contains the x, y and z values of the last target specified by the user.
    """
    resp = LastCoordResponse()
    resp.last_x = target_coord.x
    resp.last_y = target_coord.y
    resp.last_z = target_coord.z
    return resp


# Run the main
if __name__ == '__main__':
    try:
        rospy.init_node('last_target')
        sub = rospy.Subscriber('/reaching_goal/goal', assignment_2_2023.msg.PlanningActionGoal, target_callback)
        service_serv = rospy.Service('last_target_serv', LastCoord, last_target_callback)
        rospy.Rate(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Program interrupted:", file = sys.stderr)