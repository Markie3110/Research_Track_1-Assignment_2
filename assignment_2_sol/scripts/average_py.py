#! /usr/bin/env python

## @package assignment_2_sol
# \file average_py.py
# \brief A rosnode that computes the robots average speed and distance from the target.
# \author Mark Henry Dsouza
# \date 15/05/2024
#
# \details 
#
# Description: <BR>
# Implements the ROS node "average" that creates a rosservice "average_serv" which when called, 
# returns the robots average speed and distance from the target. Utilizes the data published 
# by the node "UI" on the topic "/robot_vector" to carry out the computations.
#
# Subscribes to: <BR>
#   - /robot_vector
#   - /reaching_goal/goal
#
# Publishes to: <BR>
#   - [None]
#
# Services implemented: <BR>
#   - average_serv
#
# Action Servers called: <BR>
#   - [None]


# Import the necessary libraries
import rospy
import assignment_2_2023.msg
import assignment_2_sol.msg
from assignment_2_sol.srv import *
import geometry_msgs.msg
import sys
import math


# Declare the global variables to be used throughout the program
## Contains the averaging window size derived from the launch file
global window_size 
window_size = 0 
## A list to hold the robots speeds (up to the window_size iterations)
global speed_list
speed_list = []
## A counter that keeps track of the iteration number
global count 
count = 0


# Declare the variables that hold the messages describing the robot's target and latest odometry
## A variable holding a message of type Point containing the latest robot target.
target_coord = geometry_msgs.msg.Point()
## A variable holding a message of type CustomOdom containing the latest robot odometry.
current_vector = assignment_2_sol.msg.CustomOdom()


## \brief Callback function for the subscriber to /robot_vector
#
# A callback that handles new data sent by the topic /reaching_goal/goal describing the latest robot target.
#
# \param msg A message (geometry_msg/Point) that contains the latest target sent by the user.
def target_callback(msg):
    target_coord.x = msg.goal.target_pose.pose.position.x
    target_coord.y = msg.goal.target_pose.pose.position.y


## \brief Callback function for the subscriber to /reaching_goal/goal
#
# A callback that handles new data sent by the topic /robot_vector describing the latest robot position and linear velocity.
#
# \param msg A message (assignment_2_sol/CustomOdom) containing the latest position and linear velocity of the robot.
def vector_callback(msg):
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


## \brief Callback function for average_serv service
#
# A callback that is associated with the average_serv service server that sends the robots distance from the target and its average speed up to the last window_size iterations when called upon.
#
# \param req Contains info about the request sent by a caller to the service server. Required for the function declaration.
#
# \return resp Contains the window_size, distance from the target and average speed (assignment_2_sol/Average -> Response).
def average_callback(req):
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
        ## Holds the instance of the subscriber to /robot_vector
        sub_vector = rospy.Subscriber('/robot_vector', assignment_2_sol.msg.CustomOdom, vector_callback)
        ## Holds the instance of the subscriber to /reaching_goal/goal
        sub_goal = rospy.Subscriber('/reaching_goal/goal', assignment_2_2023.msg.PlanningActionGoal, target_callback)
        ## Holds the instance of the service 'average_serv'
        service_serv = rospy.Service('average_serv', Average, average_callback)
        rospy.Rate(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Program interrupted:", file = sys.stderr)