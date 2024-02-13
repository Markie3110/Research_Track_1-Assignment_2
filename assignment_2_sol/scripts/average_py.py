#! /usr/bin/env python


# Import the necessary libraries
import rospy
import assignment_2_2023.msg
import assignment_2_sol.msg
from assignment_2_sol.srv import *
import geometry_msgs.msg
import sys
import math


# Declare the global variables to be used throughout the program
global window_size # Contains the averaging window size
window_size = 0 
global speed_list # A list to hold the robots speeds (up to the window_size iterations)
speed_list = []
global count # A counter that keeps track of the iteration number
count = 0


# Declare the variables that hold the messages describing the robot's target and latest odometry
target_coord = geometry_msgs.msg.Point()
current_vector = assignment_2_sol.msg.CustomOdom()


def target_callback(msg):
    """ 
    A callback that handles new data sent by the topic /reaching_goal/goal describing the latest robot target.

    Args:
        msg (geometry_msg/Point): A message that contains the latest target sent by the user.
    """
    target_coord.x = msg.goal.target_pose.pose.position.x
    target_coord.y = msg.goal.target_pose.pose.position.y


def vector_callback(msg):
    """
    A callback that handles new data sent by the topic /robot_vector describing the latest robot position and linear velocity.

    Args:
        msg (assignment_2_sol/CustomOdom): A message containing the latest position and linear velocity of the robot.
    """
    # Declare the global variables 
    global window_size
    global speed_list
    global count
    # Update the robot vector
    current_vector.pos_x = msg.pos_x
    current_vector.pos_y = msg.pos_y
    current_vector.vel_x = msg.vel_x
    current_vector.vel_y = msg.vel_y
    x = math.pow(current_vector.vel_x, 2)
    y = math.pow(current_vector.vel_y, 2)
    speed = x + y
    speed = math.sqrt(speed)
    # Update the speed_list with the latest value
    if (count < window_size):
        speed_list.insert(count, speed)
        count = count + 1
    else:
        count = 0


def average_callback(req):
    """ A callback that is associated with the average_serv service server that sends the robots distance from the target and its average speed up to
        the last window_size iterations when called upon.

    Args:
        req (): Contains info about the request sent by a caller to the service server. Required for the function declaration.

    Returns:
        resp (assignment_2_sol/Average -> Response): Contains the window_size, distance from the target and average speed.
    """
    # Update the global variables
    global count
    global window_size
    global speed_list
    # Compute the distance of the robot from the target
    distance_x = target_coord.x - current_vector.pos_x
    distance_y = target_coord.y - current_vector.pos_y
    distance_x = math.pow(distance_x, 2)
    distance_y = math.pow(distance_y, 2)
    distance = distance_x + distance_y
    distance = math.sqrt(distance)
    # Compute the average speed of the robot
    avg_sum = 0.0
    for i in range(0, count):
        avg_sum = avg_sum + speed_list[i]
    avg_speed = avg_sum/window_size
    # Send the response to the caller
    resp = AverageResponse()
    resp.window_size = window_size
    resp.distance = distance
    resp.average_speed = avg_speed
    return resp


# Run the main
if __name__ == '__main__':
    try:
        rospy.init_node('average')
        window_size = rospy.get_param("/window_size")
        sub_vector = rospy.Subscriber('/robot_vector', assignment_2_sol.msg.CustomOdom, vector_callback)
        sub_goal = rospy.Subscriber('/reaching_goal/goal', assignment_2_2023.msg.PlanningActionGoal, target_callback)
        service_serv = rospy.Service('average_serv', Average, average_callback)
        rospy.Rate(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Program interrupted:", file = sys.stderr)