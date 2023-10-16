#!/usr/bin/env python3
#Initialization of Libraries
#for creating our ROS program using python
import rospy
#to create  ROS Action client
import actionlib
import math
# Action Messages that will allow our robot to “navigate to the goal”
from my_robot_tutorial.msg import Navigate2DAction, Navigate2DFeedback, Navigate2DResult, Navigate2DGoal
# msg type point with three dimensional coordinates
from geometry_msgs.msg import Point
#printing the feedback message sent by the server
def feedback_callback(feedback):
    print("Distance to goal: "+str(feedback.distance_to_point))
#function definition for calling the Action Server to print the result message sent by the server
def nav_client(user_coords):
    ##defining our client instance of actionlib.SimpleActionCLient
    client = actionlib.SimpleActionClient("navigate_2D_action",Navigate2DAction)
    #wait until receives server starts running
    client.wait_for_server()
    #sending the three coordinates as a goal to robot
    point_msg = Point(x = user_coords[0], y = user_coords[1], z = user_coords[2])
    goal = Navigate2DGoal(point_msg)
    #calling function to print the distance between the goal and current location
    client.send_goal(goal,feedback_cb = feedback_callback)
    #waiti till receives result from the server
    client.wait_for_result()
    #return the result as soon as received
    return client.get_result

if __name__ == "__main__":
    try:
        #initializing action client node
        rospy.init_node("navigate_2D_action_client_node")
        #user three axis coordinates
        user_x = input("Goal x-coordinate > ")
        user_y = input("Goal y-coordinate > ")
        user_z = input("Goal z-coordinate > ")
        #concatination of three points into an array
        user_coords = [float(user_x), float(user_y), float(user_z)]
        #calling function to obtain results from server
        result = nav_client(user_coords)

        if result:
            print("Navigate Successful")
    #if any interruption occurs during program running state
    except rospy.ROSInterruptException:
        print("Program Interrupted")
