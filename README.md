Research Track 1 - Assignment 2 Solution
================================
The following repository contains the solution to the second assignment for the Research Track 1 Course, found in the Robotics Masters Programme at the University of Genoa, Italy. Within the linked repository are a series of python scripts that implement ROS nodes which interact with a predefined ROS package running a test environment on the gazebo simulator. Utilizing these nodes, a user can control the robot behaviour by: specifying a goal position the robot should move towards, stopping robot movement by cancelling a specified goal as well as be able to monitor certain semantic data of the robot.

Table of Contents
----------------------
1. [Prerequisites]()
2. [Project Contents]()
3. [How to Install]()
4. [How to run]()
5. [Code Flowcharts]()

Prerequisites
----------------------
In order to be able to run the simulator, the "assignment_2_2023" ROS package is needed. The package can be installed from the following Github repository: *https://github.com/CarmineD8/assignment_2_2023*.

Project Contents
----------------------
The project has three elementary python scripts of interest, each of which implement a ROS node with a specific functionality.

1. user_interface_py.py<br>
Implements the node "UI", that utilizes an actionclient subscribed to the actionserver "/reaching_goal" to create a user interface that allows the user to send and preempt robot goals. This node also publishes the robots position and twist to a custom topic "/robot_vector", that is utilized by the "average" node (explained below).

2. last_target_py.py<br>
Implements the rosservice "last_target" which when called, prints the latest goal position specified by the user. Obtains the latest goal via a subscription to the "/goal" topic of the "reaching_goal" actionserver.

3. average_py.py<br>
Implements another rosservice "average" which when called, returns the robots average speed and distance from the target. Utilizes the data published by the node "UI" on the topic "/robot_vector" to carry out the computations.


How to Install
----------------------
To download the repsitory's contents to your local system you can do one of the following:

1. Using git from your local system<br>
To download the repo using git simply go to your terminal and go to the src directory within your ROS workspace. Type the following command to clone the repository to your folder:
```bash
$ git clone "https://github.com/Markie3110/Research_Track_1-Assignment_2.git"
```

2. Download the .zip from Github<br>
In a browser go to the repository on Github and download the .zip file availabe in the code dropdown box found at the top right. Unzip the file to access the contents.

How to Run
----------------------
To run the solution go to robot-sim in your local system and type the following:
```bash
python3 run.py assignment.py
```

Code Flowcharts
----------------------
The solution to the assignment was broken down into steps. Actions that are carried out repeatedly were coded into several functions that are called from a main function that controls the overall robot behaviour. In general the code works by first having the robot turn clockwise and counterclockwise to note down the codes of all boxes visible to it at the moment in its internal memory, as well as to mark the box closest to it. The position of this box, which we shall call the prime box, will be the one we bring all of the other boxes to. After finding the prime, the robot searches for every box stored in its memory and transports them to the prime. Once the robot places a box at the target, the robot updates its internal memory to reflect this change. The robot has also been programmed to keep on looking for new boxes it may have missed in the intial search, as it carries out its tasks. If any such box is detected, it is added to the list. Once all the boxes the robot has come across have been transported to the prime, the program ends.  
Given below is the pseudocode for the various functions:  


### detect_boxes ###



### scan_for_closest_box ###




### detect_closest_box ###
>


### drive ###




### turn ###



### find_box ###



### move_to_target ###



### main ###

