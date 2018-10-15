#!/usr/bin/env python2

'''
@author: Hugo Sousa

Class Robot implements a simple robot with navigation and mapping
functionalities.
'''

import rospy

from geometry_msgs.msg import Twist
from math import atan2, degrees
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class Robot():
    '''
        Class constants.
    '''

    RATE = 20
    QUEUE_SIZE = 10

    # Speed constants
    kx, kt = 1, 2

    # Messages
    laserMsg = None
    odomMsg = None

    GOAL = None
    SET_FRONT = False
    ANGLE_TOLERANCE = 0.005

    # Line coefficients.
    line_a, line_b, line_c = None, None, None

    # Tolerance constants.
    MIN_OBJ_DIST = 0.6
    LINE_DIST_TOLERANCE = 0.1
    GOAL_TOLERANCE = 0.1
    MIN_DIST_IMPROVEMENT = 0.5

    # Bug 2 variables
    OBSTACLE = False
    DIST_QH = None

    # Ranges
    FRONT_LEFT = 240
    FRONT_RIGHT = 120
    RIGHT_START = 150

    def __init__(self):
        # Messages
        self.pub = rospy.Publisher('/cmd_vel', Twist,
                                   queue_size=self.QUEUE_SIZE)
        rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(self.RATE)

    def laser_callback(self, msg):
        '''
            Callback function for laser messages.
        '''

        self.laser_msg = msg
        return

    def get_robot_coordinates(self):
        '''
            Get current robot coordinates in space.

            @return: (float, float) Tuple with x and y coordinates of the
                robot.
        '''

        x = self.odom_msg.pose.pose.position.x
        y = self.odom_msg.pose.pose.position.y

        return x, y

    def euclidean_distance(p1, p2):
        '''
            Calculate the Euclidean distance between two 2D points.

            @p1: (float, float) Point 1 coordinates.
            @p2: (float, float) Point 2 coordinates.

            @return: (float) Euclidean distance between points 1 and 2.
        '''

        return sum([(p1_i - p2_i) ** 2 for (p1_i, p2_i) in zip(p1, p2)]) ** 0.5

    def get_error(self, x, y):
        '''
            Get the error calculated as the difference between current pose
            and the desired one.

            @x: (float) Desired position x coordinate.
            @y: (float) Desired position y coordinate.

            @return:
                @ex: (float) Error in x (robot's frame).
                @et: (float) Error in theta (orientation towards goal).
        '''

        xr, yr = self.get_robot_coordinates()
        yaw = self.yawFromQuaternion()
        ex = self.euclidean_distance((xr, yr), (x, y))
        theta = atan2(y - yr, x - xr)
        et = theta - yaw

        return ex, et

    def set_goal_line(self, x, y):
        '''
            Calculate the line connecting the initial and the goal position.

            @x: (float) Desired position x coordinate.
            @y: (float) Desired position y coordinate.
        '''

        x_a, y_a = self.get_robot_coordinates()

        m = (y - y_a) / (x - x_a)
        b = y - m * x

        self.line_a = - m
        self.line_b = 1
        self.line_c = - b

        return

    def get_distance_to_line(self):
        '''
            Calculate the distance between the robot and the line set
                initially.

            @return: (float) Distance between robot and the line connecting its
                initial position to the goal.
        '''

        x, y = self.get_robot_coordinates()

        return abs(self.line_a * x + self.line_b * y + self.line_c) / \
            ((self.line_a ** 2 + self.line_b ** 2) ** 0.5)

    def yawFromQuaternion(self):
        '''
            Get robot's Yaw angle from its odom orientation quaternion.

            @return: (float) Robot's yaw in radians.
        '''

        orientation = self.odomMsg.pose.pose.orientation

        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        return atan2((2.0 * (w * z + x * y)), (1.0 - 2.0 * (y * y + z * z)))

    def move(self):
        '''
            Make the robot move and explore the environment.
        '''

        self.pub.publish(self.cmd_vel)
        self.rate.sleep()
        return

    def is_blocked(self):
        '''
            Check if the robot is blocked ahead.

            @return: (bool) True, if the robot is blocked. False otherwise.
        '''

        distances = self.laser_msg.ranges[self.FRONT_RIGHT:self.FRONT_LEFT]
        ranges = [True if r <= self.MIN_OBJ_DIST else False
                  for r in distances]

        return any(ranges)

    def get_average_dist_front(self):
        '''
            Get the average distance for obstacles ahead of the robot.

            @return: (float) Average distance for obstacles ahead of the robot.
        '''

        ranges = self.laser_msg.ranges[self.FRONT_RIGHT:self.FRONT_LEFT]
        return sum(ranges) / len(ranges)

    def get_average_dist_right(self):
        '''
            Get the average distance for obstacles to the right of the robot.

            @return: (float) Average distance for obstacles to the right of the
                robot.
        '''

        ranges = self.laser_msg.ranges[:self.RIGHT_START]
        return sum(ranges) / len(ranges)

    def get_dist_to_goal(self, x, y):
        '''
            Calculate the robot's distance to the goal position.

            @x: (float) Desired position x coordinate.
            @y: (float) Desired position y coordinate.

            @return: (float) Robot's distance to goal.
        '''

        xr, yr = self.get_robot_coordinates()
        return self.euclidean_distance((xr, yr), (x, y))

    def can_turn_right(self):
        '''
            Check if the robot can turn right.

            @return: (bool) True, if the robot can turn right. False otherwise.
        '''

        ranges = [True if r > self.MIN_OBJ_DIST else False
                  for r in self.laser_msg.ranges[:self.RIGHT_START]]
        return all(ranges)

    def follow_line(self, x, y):
        '''
            Control the robot through the line connecting its initial position
            and the goal.

            @x: (float) Desired position x coordinate.
            @y: (float) Desired position y coordinate.
        '''

        vel = Twist()
        ex, et = self.get_error(x, y)

        if not self.SET_FRONT:
            if abs(et) < self.ANGLE_TOLERANCE:
                self.SET_FRONT = True
            else:
                vel.linear.x = 0
                vel.angular.z = self.kt * et

                if abs(degrees(et)) > 180:
                    vel.angular.z = self.kt * et * -1
        else:
            if self.is_blocked():
                self.OBSTACLE = True
                self.DIST_QH = self.get_dist_to_goal()
                self.SET_FRONT = False

                vel.linear.x = 0
                vel.angular.z = 0
                self.cmd_vel = vel
                self.move()
                return

            vel.linear.x = self.kx * abs(ex)

        self.cmd_vel = vel
        self.move()
        return

    def outline_obstacle(self, x, y):
        '''
            Control the robot around an obstacle.

            @x: (float) Desired position x coordinate.
            @y: (float) Desired position y coordinate.
        '''

        vel = Twist()

        # Check if robot can move to GOAL again.
        dist_line = self.get_distance_to_line()
        dist_goal = self.get_dist_to_goal(x, y)
        if dist_line < self.LINE_DIST_TOLERANCE and \
                dist_goal + self.MIN_DIST_IMPROVEMENT < self.DIST_QH:

            self.OBSTACLE = False
            vel.linear.x = 0
            vel.angular.z = 0
            self.cmd_vel(vel)
            self.move()
            return

        if self.is_blocked():  # Turning left
            vel.angular.z = self.kt
            self.cmd_vel(vel)
            self.move()
            return
        else:
            vel.linear.x = self.kx

            if self.can_turn_right():  # Turning right
                right_distance = self.get_average_dist_right()
                vel.linear.x = self.kx / right_distance
                vel.angular.z = - self.kt * right_distance
                self.cmd_vel(vel)
                self.move()
                return

            self.cmd_vel(vel)
            self.move()
            return

    def bug2(self, x, y):
        '''
            Control the robot towards the goal position, using the Bug 2
            algorithm.

            @x: (float) Desired position x coordinate.
            @y: (float) Desired position y coordinate.
        '''

        if self.OBSTACLE:
            self.outline_obstacle(x, y)

        self.follow_line(x, y)
