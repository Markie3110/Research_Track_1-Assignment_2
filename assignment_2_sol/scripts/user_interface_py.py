#! /usr/bin/env python



# Import the necessary libraries
import rospy
import actionlib
import assignment_2_2023.msg
import assignment_2_sol.msg
import nav_msgs.msg
import sys
import select
import time



# Declare the strings to be used to display the interface
msg_interface = """-----------------------------------------------------------------------
SIMPLE ROBOT INTERFACE FOR GAZEBO
---------------------------------
Type in the number corresponding to the desired action and press ENTER:
1 - Set Robot Target
2 - Quit
---------------------------------
Input: """
msg_close = """---------------------------------
Closing interface 
GOODBYE!!"""
msg_error = """---------------------------------
The following input is invalid (including CTRL+C). Please choose between 1 and 2."""



# Initialize the publisher for the custom odom
pub = rospy.Publisher('/robot_vector', assignment_2_sol.msg.CustomOdom, queue_size=15)
# Declare a variable to hold the message to be published to /robot_vector
robot_odom = assignment_2_sol.msg.CustomOdom()


def odom_callback(msg):
    """ A callback that handles new data received from the topic /odom.

    Args:
        msg (nav_msgs/Odometry): A message that contains the latest position and velocity values of the robot.
    """
    robot_odom.pos_x = msg.pose.pose.position.x
    robot_odom.pos_y = msg.pose.pose.position.y
    robot_odom.vel_x = msg.twist.twist.linear.x
    robot_odom.vel_y = msg.twist.twist.linear.y
    pub.publish(robot_odom)



# Declare a Client class
class UIClient:
    """ A class that describes the actionlib clients characteristics and behaviour.
    """


    # Declare the required class variables
    preempt = False # A flag that tracks if a preempt was called
    run = True # A flag that tracks whether the user has requested to close this program


    # Declare the target variable
    target = assignment_2_2023.msg.PlanningGoal()


    def __init__(self):
        """ A method that executes when a new instance of the class is intialized.
        """

        # Initialize the action client
        self.client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
        rospy.loginfo("Started UI client")
        # Wait for the client to connect to the action server
        self.client.wait_for_server()
        rospy.loginfo("Connected to the server")
        # Run the interface
        while(self.run):
            # Print the intro message for the interface
            print(msg_interface, end="")
            inp = sys.stdin.readline().strip()
            if(inp == '1'):
                # Initalize the target
                print("---------------------------------")
                self.target.target_pose.pose.position.x = float(input("Target_x: "))
                self.target.target_pose.pose.position.y = float(input("Target_y: "))
                # Send the goal to the action server
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
        

    # Declare the feedback callback
    def feedback_cb(self, feedback):
        """ A callback that is executed everytime the client receives a feedback from the server.

        Args:
            feedback (assignment_2_2023/PlanningFeedback): A message that contains the latest pose of the robot as it moves towards the goal.
        """

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


    # Declare the done callback
    def done_cb(self, state, result):
        """A callback that is executed when the robot reaches its goal.

        Args:
            state (): A message describing the state of the robot when it reaches the goal.
            result (assignment_2_2023/PlanningResult): An empty message (as the result is not defined in assignment_2_2023/action/Planning.action). '
                                                       Required for the function declaration.
        """

        if (self.preempt == False):
            print("[UPDATE] ROBOT HAS REACHED THE TARGET") 
        else:
            print("[UPDATE] ROBOT PREEMPTED")
            self.preempt = False
        time.sleep(3)
        


# Run the main
if __name__ == '__main__':
    try:
        # Wait for gazebbo to launch completely
        time.sleep(8)
        # Initialize the ros node
        rospy.init_node('UI')
        # Subscribe to the topic /odom
        sub = rospy.Subscriber('/odom', nav_msgs.msg.Odometry, odom_callback)
        # Run the action client
        ui_client = UIClient()
        del ui_client
        rospy.loginfo("Closing server")
    except rospy.ROSInterruptException:
        print("Program interrupted:", file = sys.stderr)