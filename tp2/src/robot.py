#!/usr/bin/env python2

'''
@author: Hugo Sousa

Class Robot implements a simple robot with navigation and mapping
functionalities.
'''

import numpy as np
import rospy

from geometry_msgs.msg import Twist
from grid import Grid
from math import atan2, degrees
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

np.warnings.filterwarnings('ignore')


class Robot:
    '''
        Class constants.
    '''

    # Speed constants
    kx, kt = 1, 2

    # Tolerance constants.
    MIN_OBJ_DIST = 0.8
    DIST_TOLERANCE = 0.2
    MIN_DIST_IMPROVEMENT = 0.5
    ANGLE_TOLERANCE = 0.005

    # Ranges
    FRONT_LEFT = 240
    FRONT_RIGHT = 120
    RIGHT_START = 150

    def __init__(self, robot_rate, ros_queue_size):
        # Navigation variables
        self.set_front = False
        self.obstacle = False
        self.dist_qh = None
        self.hit_point = None
        self.out_of_hit_point = None

        # Messages
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist,
                                       queue_size=ros_queue_size)
        rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/base_pose_ground_truth',
                         Odometry, self.odom_callback)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid,
                                       queue_size=ros_queue_size)
        self.rate = rospy.Rate(robot_rate)

        self.laser_msg = None
        self.odom_msg = None

        # Line coefficients.
        self.line_a, self.line_b, self.line_c = None, None, None
        self.line_set = False

    def is_ready(self):
        '''
            Check if robot has all needed messages.

            @return: (bool) True iff robot has all messages it needs.
        '''

        return self.laser_msg is not None and self.odom_msg is not None

    def laser_callback(self, msg):
        '''
            Callback function for laser messages.
        '''

        self.laser_msg = msg
        return

    def odom_callback(self, msg):
        '''
            Callback function for odom messages.
        '''

        self.odom_msg = msg
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

    def euclidean_distance(self, p1, p2):
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
        yaw = self.get_yaw()
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

    def get_distance_to_line(self, x, y):
        '''
            Calculate the distance between the robot and the line set
                initially.

            @x: (float) Desired position x coordinate.
            @y: (float) Desired position y coordinate.

            @return: (float) Distance between robot and the line connecting its
                initial position to the goal.
        '''

        coord = self.get_robot_coordinates()

        return abs(self.line_a * coord[0] + self.line_b * coord[1] +
                   self.line_c) / ((self.line_a ** 2 +
                                    self.line_b ** 2) ** 0.5)

    def get_yaw(self):
        '''
            Get robot's Yaw angle from its odom orientation quaternion.

            @return: (float) Robot's yaw in radians.
        '''

        orientation = self.odom_msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z,
                      orientation.w)
        return euler_from_quaternion(quaternion)[2]

    def move(self):
        '''
            Make the robot move and explore the environment.
        '''

        self.vel_pub.publish(self.cmd_vel)
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

        if not self.set_front:
            if abs(et) < self.ANGLE_TOLERANCE:
                self.set_front = True
            else:
                vel.linear.x = 0
                vel.angular.z = self.kt * et

                if abs(degrees(et)) > 180:
                    vel.angular.z = self.kt * et * -1
        else:
            if self.is_blocked():
                self.obstacle = True
                self.hit_point = self.get_robot_coordinates()
                self.dist_qh = self.get_dist_to_goal(x, y)
                self.set_front = False

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
        dist_line = self.get_distance_to_line(x, y)
        dist_goal = self.get_dist_to_goal(x, y)
        dist_hit_point = self.get_dist_to_goal(*self.hit_point)

        # Check if robot has left the hit point.
        # This is necessary to asses if a goal is unreachable, when the robot
        # gets to the same hit point again.
        if self.out_of_hit_point is None:
            if dist_hit_point > self.MIN_DIST_IMPROVEMENT:
                self.out_of_hit_point = True

        if self.out_of_hit_point:
            if dist_hit_point < self.DIST_TOLERANCE:
                self.obstacle = False
                self.out_of_hit_point = None
                vel.linear.x = 0
                vel.angular.z = 0
                self.cmd_vel = vel
                self.move()
                return False

        if dist_line < self.DIST_TOLERANCE and \
                dist_goal + self.MIN_DIST_IMPROVEMENT < self.dist_qh:

            self.obstacle = False
            vel.linear.x = 0
            vel.angular.z = 0
            self.cmd_vel = vel
            self.move()
            return True

        if self.is_blocked():  # Turning left
            vel.angular.z = self.kt
            self.cmd_vel = vel
            self.move()
            return True
        else:
            vel.linear.x = self.kx

            if self.can_turn_right():  # Turning right
                right_distance = self.get_average_dist_right()
                vel.linear.x = self.kx / right_distance
                vel.angular.z = - self.kt * right_distance
                self.cmd_vel = vel
                self.move()
                return True

            self.cmd_vel = vel
            self.move()
            return True

    def bug2(self, x, y):
        '''
            Control the robot towards the goal position, using the Bug 2
            algorithm.

            @x: (float) Desired position x coordinate.
            @y: (float) Desired position y coordinate.
        '''

        # Robot reached goal.
        if self.get_dist_to_goal(x, y) < self.DIST_TOLERANCE:
            # Line coefficients.
            self.line_a, self.line_b, self.line_c = None, None, None
            self.line_set = False
            return False

        # Set line to goal and follow it.
        if not self.line_set:
            self.set_goal_line(x, y)
            self.line_set = True

        # Robot is blocked by obstacle.
        if self.obstacle:
            if self.outline_obstacle(x, y):
                return True
            else:
                self.line_a, self.line_b, self.line_c = None, None, None
                self.line_set = False
                return False

        self.follow_line(x, y)
        return True

    def apply_measurements_to_grid(self, grid):
        '''
            Update the grid map with the new measurements from laser sensor.

            @grid: (Grid) Current Occupancy Grid object.
        '''

        pos = np.array(self.get_robot_coordinates())

        # Array with distances from robot to center of mass of all cells.
        cell_distances = np.asarray([[np.linalg.norm(np.array(
            grid.center_of_mass_from_index(i, j) - pos))
            for i in range(grid.width)] for j in range(grid.height)])

        # Array with angles between the robot's orientation and the center of
        # mass of all cells.
        cell_angles = np.asarray([[np.arctan2(*tuple(
            grid.center_of_mass_from_index(i, j)[::-1] - pos[::-1]))
            for i in range(grid.width)] for j in range(grid.height)])

        yaw = self.get_yaw()
        for i in range(len(self.laser_msg.ranges)):
            d = self.laser_msg.ranges[i]  # Observed range
            # Angle between robot and the end of beam.
            theta = np.radians(i / 2) - np.radians(90) + yaw

            # Free cells are closer to the robot but still within the beam
            # angle.
            free_cells = np.logical_and(
                np.abs(cell_angles - theta) <= (grid.beta / 2),
                (cell_distances < (d - grid.ALPHA / 2)))

            # Occupied cells are further from the robot but still within the
            # beam angle.
            occupied_cells = np.logical_and(
                np.abs(cell_angles - theta) <= (grid.beta / 2),
                (np.abs(cell_distances - d) <= grid.ALPHA / 2))

            # Update the log-odds ratio for all cells accordingly to their
            # type (free or occupied).
            grid.cells[free_cells] += grid.l_free
            grid.cells[occupied_cells] += grid.l_occ

        return

    def publish_occupancy_grid(self, grid):
        '''
            Create and publish Occupancy Grid message.

            @grid: (Grid) Current Occupancy Grid object.
        '''

        # Create and publish message.
        msg = grid.get_occupancy_msg()
        self.map_pub.publish(msg)
        return

    def occupancy_grid(self, height, width, resolution, frontiers):
        '''
            Run the Occupancy Grid algorithm to find the environment map.

            @height: (int) Number of cells that form the grid's height.
            @width: (int) Number of cells that form the grid's width.
            @resolution: (float) Size of the cells side.
            @frontiers: (int) Number of random frontier points to explore.
        '''

        max_laser_range = 8.0
        num_ranges = len(self.laser_msg.ranges)
        grid = Grid(height, width, resolution, max_laser_range, num_ranges)

        goal = None
        frontiers_explored = 0
        while not rospy.is_shutdown():
            # Occupancy Grid algorithm.
            self.apply_measurements_to_grid(grid)
            self.publish_occupancy_grid(grid)

            # Exploration
            if goal is None or not self.bug2(*goal):
                goal = grid.get_random_frontier_cell()
                frontiers_explored += 1

                if goal is None or frontiers_explored > frontiers:
                    break

            self.rate.sleep()

        grid.dump_pgm()
