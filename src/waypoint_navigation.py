#! /usr/bin/env python
# waypoint_navigation.py
# Joey Grossman & Harmeet Singh 2025

import copy
import actionlib
import rospy

from math import sin, cos, pi
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

class InitialPose:

    def __init__(self):
        # Starts the initial pose publisher
        self.initial_pose_publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped,  queue_size=0)
        rospy.sleep(1)
        # Starts the velocity publisher
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.sleep(1)
        rospy.loginfo('Initial Pose Publisher started')

    # Publishes the initial pose estimate so we don't need to manually do it
    def publish_initial_pose(self, x, y, theta):

        msg = PoseWithCovarianceStamped()   # Msg used for setting initial position estiate
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'

        # Sets the parameters initial position
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = sin(theta/2.0)
        msg.pose.pose.orientation.w = cos(theta/2.0)

        # Apparently needed for AMCL reasons
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        # Publishes the initial pose
        self.initial_pose_publisher.publish(msg)
        rospy.loginfo('Published initial pose')
        rospy.sleep(1)

    # Rotates the robot to get a better initial estimate of where the robot is
    def rotate(self):
        rospy.loginfo('Roating to get a better estimate of location')
        vel_msg = Twist()

        angular_speed = 90.0*2*pi/360.0
        relative_angle = 4*pi

        # Linear velocity is unused
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0

        # Rotates clockwise
        vel_msg.angular.z = -abs(angular_speed)

        # Initial time
        t0 = rospy.Time.now().to_sec()

        # Initial rotation
        current_angle = 0


        # Continues until the robot reaches the relative angle
        while(current_angle < relative_angle):
            self.vel_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        # Stops the robot from rotating
        vel_msg.angular.z = 0
        self.vel_publisher.publish(vel_msg)
        rospy.loginfo('Finished estimating location')
        rospy.sleep(1)






if __name__ == "__main__":
    rospy.init_node("waypoint_navigation")
    rospy.sleep(1)

    # dictionary storing current locations as a xy coordinate ignoring z as we will use 0 for that each time
    locations = {0 : (52.676197052, 2.41499710083),
                 1 : (66.4334030151, 7.99283123016),
                 2 : (59.5403862, 8.19268226624),
                 3 : (63.5038337708, 12.7281942368),
                 4 : (47.4287071228, 9.14681243896),
                 5 : (37.0003814697, 9.91230010986),
                 6 : (36.3330001831, 12.8870811462),
                 7 : (34.1403961182, 18.8323631287),
                 8 : (30.0102310181, 11.7271757126),
                 9 : (4.02810621262, 14.8481292725),
                 10 : (23.0405063629,13.4057121277),
                 11 : (12.501241684, 14.2192783356),
                 12 : (32.1649627686, 24.7574310303),
                 13 : (22.1733093262, 6.98352813721)  
                }

    # used to terminate program
    done = False

    # unsure on this since this was copied from the original file
    while not rospy.Time.now():
        pass
    
    # Sets the initial position and rotates to get a better estimate of location
    initial_pose_publisher = InitialPose()
    initial_pose_publisher.publish_initial_pose(32.0709686279,24.8866462708, 0.0) # set the initial location to kitchen area
    initial_pose_publisher.rotate()
    rospy.sleep(1)

    # sets move_base to the client from the above method
    move_base = MoveBaseClient()
    
    while not done:

        # get location using input (using temp locations in the hallway by the lab until we have a full list)
        print("0 : Elevators"
              "1: Room 401 'Collaborative' \n "
              "2: Room 402 'Levy Family Seminar Room' \n"
              "3: Room 481 'PHD Offices' \n"
              "4: Room 405 'Utility Corridor' \n"
              "5: Room 420 'Computer Science Admin Office' \n"
              "6: Room 474 'MuLIP Lab' \n"
              "7: Room 472 'AABL Lab' \n"
              "8: 'Kitchenette' \n"
              "9: 'Huddle' \n"
              "10: Room 435 'Teaching Lab' \n"
              "11: 'Bathrooms'\n"
              "12: 'General Offices near Labs' \n"
              "13: 'General Offices near Huddle'\n"
              )
        location = input("Please enter the number that corresponds with your destination: ")
        # stores the tuple in coordinates
        coordinates = locations[location]

        rospy.loginfo("Moving")
        # use the goto function using the x and y from coordinates variable and then 0 for z as we might not need that every time
        move_base.goto(coordinates[0],coordinates[1],0)

        final = input("If this is your final destination please enter 1. Otherwise enter 2: ")

        if final == 1:
            done = True

        else:
            done = False
    print("Thank you for using Tufts AABL's Boop!")
