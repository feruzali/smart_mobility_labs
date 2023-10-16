#!/usr/bin/env python3
#Initiallization of Libraries
#for creating our ROS program using python
import rospy
#to create  ROS Action Server
import actionlib
import math
# Action Messages that will allow our robot to “navigate to the goal”
from my_robot_tutorial.msg import Navigate2DAction, Navigate2DFeedback, Navigate2DResult
# msg type point with three dimensional coordinates
from geometry_msgs.msg import Point
#class definition
class Navigate2DClass():
    def __init__(self):
        #defining our action_server instance of actionlib.SimpleActionServer with specification of action and
        #callback function that will be executed once a client calls our server
        self.action_server = actionlib.SimpleActionServer("navigate_2D_action", Navigate2DAction, self.navigate_cb)
        #subscribing to the topic 'robot/point' to get the goal location
        self.robot_point_sub = rospy.Subscriber("robot/point", Point, self.update_robot_position)
        #defining the variables
        self.robot_current_point = None
        self.robot_goal_point = None
        #threshold as a speed for robot to move and stops if reaches this limit
        self.distance_threshold = 0.35
        #feedback after every one second
        self.feedback_rate = rospy.Rate(1)
    #defining function that executes with the user provided goal
    def navigate_cb(self, goal):
        self.robot_goal_point = [goal.point.x, goal.point.y, goal.point.z]
        navigate_start_time = rospy.get_time()
        #until no data is provided by the user
        while self.robot_current_point == None:
            print("Robot Point Not Detected")
            rospy.sleep(2)
        #once data is received by the user
        print("Robot Point Detected")
        # distance calculation of robot current point to its goal
        distance_to_goal = math.dist(self.robot_current_point, self.robot_goal_point)
        #robot will move towards goal untill reach its threshold limit
        while distance_to_goal > self.distance_threshold:
            # providing a feedback about distance to its goal
            self.action_server.publish_feedback(Navigate2DFeedback (distance_to_point = distance_to_goal - self.distance_threshold))
            # after every interval, robot is moving towards its destination with the threshold speed
            distance_to_goal =  distance_to_goal - self.distance_threshold #math.dist(self.robot_current_point, self.robot_goal_point)

            rospy.sleep(2)
        # reading time for robot to reach destination
        navigate_end_time = rospy.get_time()
        elapsed_time = navigate_end_time - navigate_start_time
        rospy.loginfo("Navigate Successful, Elapsed Time: " +str(elapsed_time)+ "s")
        #on reaching the goal, the program will stop and task completion time will be sent to user
        self.action_server.set_succeeded(Navigate2DResult(elapsed_time))
    #obtaining the current position of robot
    def update_robot_position(self,point):
        self.robot_current_point = [point.x, point.y, point.z]
#main function of the script
if __name__ == "__main__":
    #initializing action server node
    rospy.init_node("navigate_2D_action_server_node")
    #calling the class
    server = Navigate2DClass()
    rospy.spin()
