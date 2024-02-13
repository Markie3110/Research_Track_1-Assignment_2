Research Track 1 - Assignment 2 Solution
================================
The following repository contains the solution to the first assignment for the Research Track 1 Course, found in the Robotics Masters Programme at the University of Genoa, Italy. The problem statement along with any 
necessary files can be found at *https://github.com/CarmineD8/python_simulator*. Within the linked repository is a simulator that is capable of depicting a differential drive robot and some boxes in an arena. The goal 
of the assignment is to program the robot with the help of some predefined functions such that it can detect all the boxes within the arena space, and move them to a single position. The position can be arbitary and it 
is for the programmer to decide how they want to carry out the execution.

Table of Contents
----------------------
1. [Project Contents]()
2. [How to Install]()
3. [How to run]()
5. [Code Flowcharts]()

How to Install
----------------------
To download the repsitory's contents to your local system you can do one of the following:

1. Using git from your local system
To download the repo using git simply go to your terminal and go to the directory you want to save the project in. Type the following command to clone the repository to your local folder:
```bash
$ git clone "https://github.com/Markie3110/Research_Track_1-Assignment_2.git"
```

2. Download the .zip from Github
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

