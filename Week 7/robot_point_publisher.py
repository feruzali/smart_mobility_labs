#!/usr/bin/env python3
#Initialization of libraries
import rospy
#importing Point type msg from the package
from geometry_msgs.msg import Point
#function definition to publish current point coordinates
def robot_point_pub(user_coords):
    #creating a topic 'robot/point' for publishing current point axis
    pub = rospy.Publisher("robot/point", Point, queue_size =10)
    print("Publishing")
    rate = rospy.Rate(1)
    #until system is in running state
    while not rospy.is_shutdown():
        #publihsing three dimensional coordinates of the current robot point
        pub.publish(Point(x=user_coords[0], y=user_coords[1], z=user_coords[2]))
        rate.sleep()
#main function of the script
if __name__ =="__main__":
    try:
        #initializing publisher node
        rospy.init_node("robot_point_pub_node")
        user_x = input("Current x-coordinate > ")
        user_y = input("Current y-coordinate > ")
        user_z = input("Current z-coordinate > ")
        #concatinating three coordinates into an array
        user_coords = [float(user_x), float(user_y), float(user_z)]
        robot_point_pub(user_coords)
    #if any error/exception occurs during program running
    except rospy.ROSInterruptException:
        print("Exception Occured")
