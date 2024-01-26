#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sqrt, pow
import sys

class TurtleBotController:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)

        self.pose = None

    def update_pose(self, data):
        self.pose = data.pose.pose

    def get_distance(self, goal_x, goal_y):
        current_x = self.pose.position.x
        current_y = self.pose.position.y

        distance = sqrt(pow((goal_x - current_x), 2) + pow((goal_y - current_y), 2))
        return distance
    

    def move_turtlebot(self, goal_x, goal_y):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
        # Wait until pose information is received
            while self.pose is None and not rospy.is_shutdown():
                rospy.loginfo("Waiting for odometry information...")
                rate.sleep()

            if rospy.is_shutdown():
                break

            distance = self.get_distance(goal_x, goal_y)

            if distance < 0.1:
                rospy.loginfo("Goal reached!")
                self.stop_movement()
                break

            goal_angle = atan2(goal_y - self.pose.position.y, goal_x - self.pose.position.x)

            if abs(goal_angle - self.get_orientation()) > 0.1:
                self.rotate_turtlebot(goal_angle)
            else:
                self.stop_movement()
                self.linear_turtlebot()

            rate.sleep()


    def stop_movement(self):
        twist = Twist()
        twist.angular.z = 0
        self.velocity_publisher.publish(twist)

    def get_orientation(self):
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        return yaw

    def rotate_turtlebot(self, goal_angle):
        twist = Twist()
        twist.angular.z = 0.15 if goal_angle > self.get_orientation() else -0.15
        self.velocity_publisher.publish(twist)

    def linear_turtlebot(self):
        twist = Twist()
        twist.linear.x = 0.15
        self.velocity_publisher.publish(twist)

if __name__ == '__main__':
    try:
        controller = TurtleBotController()

        # Read goal coordinates from a file
        
        if len(sys.argv) > 1:
            filename = sys.argv[1]
        else:
            filename = "rrt_nodes.txt"

        with open(filename, 'r') as file:
            lines = file.readlines()
            goals = [(float(line.split()[0]), float(line.split()[1])) for line in lines]

        # Move to each goal sequentially
        for goal_x, goal_y in goals:
            controller.move_turtlebot(goal_x, goal_y)

    except rospy.ROSInterruptException:
        pass

