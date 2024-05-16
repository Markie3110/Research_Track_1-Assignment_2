#! /usr/bin/env python

## @package assignment_2_sol
# \file user_interface_py.py
# \brief A user interface for the turtlesim controller
# \author Mark Henry Dsouza
# \date 15/05/2024
#
# \details 
#
# Description: <BR>
# Implements the node "UI", that utilizes an action-client subscribed to the action-server 
# "/reaching_goal" to create a user interface that allows the user to send and preempt 
# robot goals. This node also publishes the robots position and twist to a custom topic 
# "/robot_vector", that is utilized by the "average" node .
#
# Subscribes to: <BR>
#   - /odom
#
# Publishes to: <BR>
#   - /robot_vector
#
# Services implemented: <BR>
#   - [None]
#
# Action Servers called: <BR>
#   - /reaching_goal

## Necessary libraries
import rospy
import actionlib
import assignment_2_2023.msg
import assignment_2_sol.msg
import nav_msgs.msg
import sys
import select
import time


## String to be displayed at the start in the UI
msg_interface = """-----------------------------------------------------------------------
SIMPLE ROBOT INTERFACE FOR GAZEBO
---------------------------------
Type in the number corresponding to the desired action and press ENTER:
1 - Set Robot Target
2 - Quit
---------------------------------
Input: """

## String to be displayed when closing the UI
msg_close = """---------------------------------
Closing interface 
GOODBYE!!"""

## String to be displayed when an invalid input is received
msg_error = """---------------------------------
The following input is invalid (including CTRL+C). Please choose between 1 and 2."""

## Holds the instance of the publisher to /robot_vector
pub = rospy.Publisher('/robot_vector', assignment_2_sol.msg.CustomOdom, queue_size=15)
## Holds the message to be published to /robot_vector of type CustomOdom
robot_odom = assignment_2_sol.msg.CustomOdom()

## 
# \brief A callback for the subscriber to /odom.
#
# A callback function for the subscriber to the topic /odom. Receives the latest positional 
# and velocity updates of the turtlesim robot from msg, updates the local variable robot_odom
# holding the CustomOdom message and publishes it to /robot_vector.
#
# \param msg A message (nav_msgs/Odometry) that contains the latest position and velocity values of the robot.
# \return None
# 
def odom_callback(msg):
    robot_odom.pos_x = msg.pose.pose.position.x
    robot_odom.pos_y = msg.pose.pose.position.y
    robot_odom.vel_x = msg.twist.twist.linear.x
    robot_odom.vel_y = msg.twist.twist.linear.y
    pub.publish(robot_odom)



## \class UIClient
# \brief A class that describes the actionlib clients characteristics and behaviour.
#
# Governs the interaction between the user interface and the action client calling to
# the /reaching_goal action server.
class UIClient:

    ## \brief Constructor function
    #
    # Initializes the action client and implements a user interface that allows the user to
    # set a target for the turtlesim bot.
    #
    # \param self The object pointer
    def __init__(self):
        ## A flag that tracks if a preempt was called
        self.preempt = False 
        ## A flag that tracks whether the user has requested to close this program
        self.run = True 
        ## Holds the desired target position to be sent to the action server
        self.target = assignment_2_2023.msg.PlanningGoal()
        ## Holds an instance of the action client
        self.client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
        rospy.loginfo("Started UI client")
        self.client.wait_for_server()
        rospy.loginfo("Connected to the server")
        while(self.run):
            print(msg_interface, end="")
            inp = sys.stdin.readline().strip()
            if(inp == '1'):
                print("---------------------------------")
                self.target.target_pose.pose.position.x = float(input("Target_x: "))
                self.target.target_pose.pose.position.y = float(input("Target_y: "))
                self.client.send_goal(self.target, done_cb=self.done_cb, feedback_cb=self.feedback_cb)
                self.client.wait_for_result()
            elif (inp == '2'):
                print(msg_close)
                self.run = False
            elif (inp == ''):
                print(' ')
                continue
            else:
                print(msg_error)
                time.sleep(3)
        

    ## 
    # \brief The feedback callback for the action-client to /reaching_goal
    #
    # Prints the feedback from the action server onto the terminal. Checks if the user has 
    # preempted the goal and cancels it accordingly.
    #
    # \param self The object pointer
    # \param feedback A message (assignment_2_2023/PlanningFeedback) that contains the latest pose of the robot as it moves towards the goal.
    def feedback_cb(self, feedback):
        print("---------------------------------")
        print("[STATE] {0}".format(feedback.stat))
        print("[FEEDBACK] |{0}, {1}|".format(feedback.actual_pose.position.x, feedback.actual_pose.position.y))
        print("[NOTE] Press q+ENTER to preempt the goal")
        [a, b, c] = select.select([sys.stdin], [], [], 1)
        if(a):
            c = sys.stdin.read(1)
            if (c == 'q'):
                self.preempt = True
                self.client.cancel_goal()


    ## 
    # \brief The goal feedback for the action-client to /reaching-goal.
    #
    # Prints a message on the terminal updating the user on whether the robot has reached the goal or was preempted.
    #
    # \param self The object pointer
    # \param state (): A message describing the state of the robot when it reaches the goal.
    # \param result An empty message (assignment_2_2023/PlanningResult). Empty as the result is not defined in assignment_2_2023/action/Planning.action. Required for the function declaration.
    def done_cb(self, state, result):
        if (self.preempt == False):
            print("[UPDATE] ROBOT HAS REACHED THE TARGET") 
        else:
            print("[UPDATE] ROBOT PREEMPTED")
            self.preempt = False
        time.sleep(3)
        


# Run the main
if __name__ == '__main__':
    try:
        time.sleep(8)
        rospy.init_node('UI')
        ## Holds the instance of the subscriber to /odom
        sub = rospy.Subscriber('/odom', nav_msgs.msg.Odometry, odom_callback)
        ## Holds an instance of the UIClient class 
        ui_client = UIClient()
        del ui_client
        rospy.loginfo("Closing server")
    except rospy.ROSInterruptException:
        print("Program interrupted:", file = sys.stderr)