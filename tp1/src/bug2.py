#!/usr/bin/env python2

import rospy

from geometry_msgs.msg import Twist
from math import atan2, degrees
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sys import argv
from time import time

RATE = 20
QUEUE_SIZE = 10

# Speed constants
kx, ky, kt = 1, 1, 2

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


def LaserCallback(msg):
    '''
        Callback function for laser messages.
    '''

    global laserMsg

    laserMsg = msg
    return


def OdomCallback(msg):
    '''
        Callback function for odometry messages.
    '''

    global odomMsg

    odomMsg = msg
    return


def get_robot_coordinates():
    '''
        Get current robot coordinates in space.

        @return: (float, float) Tuple with x and y coordinates of the robot.
    '''

    x = odomMsg.pose.pose.position.x
    y = odomMsg.pose.pose.position.y

    return x, y


def yawFromQuaternion(orientation):
    '''
        Get robot's Yaw angle from its odom orientation quaternion.

        @orientation: (odom orientation) Robot current quaternion.

        @return: (float) Robot's yaw in radians.
    '''

    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    return atan2((2.0 * (w * z + x * y)), (1.0 - 2.0 * (y * y + z * z)))


def get_error():
    '''
        Get the error calculated as the difference between current pose and the
        desired one.

        @return:
            @ex: (float) Error in x (robot's frame).
            @et: (float) Error in theta (orientation towards goal).
    '''

    global theta

    x, y = get_robot_coordinates()
    yaw = yawFromQuaternion(odomMsg.pose.pose.orientation)
    ex = get_dist_to_goal()
    theta = atan2(GOAL[1] - y, GOAL[0] - x)
    et = theta - yaw

    return ex, et


def set_params():
    '''
        Set the input parameters.
    '''

    global GOAL

    GOAL = float(argv[1]), float(argv[2])
    return


def set_goal_line():
    '''
        Calculate the line connecting the initial and the goal position.
    '''

    global line_a, line_b, line_c

    x_a, y_a = get_robot_coordinates()
    x_b, y_b = GOAL

    m = (y_b - y_a) / (x_b - x_a)
    b = y_b - m * x_b

    line_a = - m
    line_b = 1
    line_c = - b
    return


def get_distance_to_line():
    '''
        Calculate the distance between the robot and the line set initially.

        @return: (float) Distance between robot and the line connecting its
            initial position to the goal.
    '''

    x, y = get_robot_coordinates()

    return abs(line_a * x + line_b * y + line_c) / \
        ((line_a ** 2 + line_b ** 2) ** 0.5)


def euclidean_distance(p1, p2):
    '''
        Calculate the Euclidean distance between two 2D points.

        @p1: (float, float) Point 1 coordinates.
        @p2: (float, float) Point 2 coordinates.

        @return: (float) Euclidean distance between points 1 and 2.
    '''

    return sum([(p1_i - p2_i) ** 2 for (p1_i, p2_i) in zip(p1, p2)]) ** 0.5


def follow_line():
    '''
        Control the robot through the line connecting its initial position and
        the goal.

        @return: (Twist) Velocity object.
    '''

    global DIST_QH, OBSTACLE, SET_FRONT

    vel = Twist()
    ex, et = get_error()

    if not SET_FRONT:
        if abs(et) < ANGLE_TOLERANCE:
            SET_FRONT = True
        else:
            vel.linear.x = 0
            vel.angular.z = kt * et

            if abs(degrees(et)) > 180:
                vel.angular.z = kt * et * -1
    else:
        if is_blocked():
            OBSTACLE = True
            DIST_QH = get_dist_to_goal()
            SET_FRONT = False
            vel.linear.x = 0
            vel.angular.z = 0
            return vel

        vel.linear.x = kx * abs(ex)

    return vel


def is_blocked():
    '''
        Check if the robot is blocked ahead.

        @return: (bool) True, if the robot is blocked. False otherwise.
    '''

    ranges = [True if r <= MIN_OBJ_DIST else False
              for r in laserMsg.ranges[FRONT_RIGHT:FRONT_LEFT]]

    return any(ranges)


def get_average_dist_front():
    '''
        Get the average distance for obstacles ahead of the robot.

        @return: (float) Average distance for obstacles ahead of the robot.
    '''

    ranges = laserMsg.ranges[FRONT_RIGHT:FRONT_LEFT]

    return sum(ranges) / len(ranges)


def get_average_dist_right():
    '''
        Get the average distance for obstacles to the right of the robot.

        @return: (float) Average distance for obstacles to the right of the
            robot.
    '''

    ranges = laserMsg.ranges[:RIGHT_START]

    return sum(ranges) / len(ranges)


def can_turn_right():
    '''
        Check if the robot can turn right.

        @return: (bool) True, if the robot can turn right. False otherwise.
    '''

    ranges = [True if r > MIN_OBJ_DIST else False
              for r in laserMsg.ranges[:RIGHT_START]]

    return all(ranges)


def get_dist_to_goal():
    '''
        Calculate the robot's distance to the goal position.

        @return: (float) Robot's distance to goal.
    '''

    x, y = get_robot_coordinates()
    return euclidean_distance((x, y), GOAL)


def outline_obstacle():
    '''
        Control the robot around an obstacle.

        @return: (Twist) Velocity object.
    '''

    global OBSTACLE

    vel = Twist()

    # Check if robot can move to GOAL again.
    if get_distance_to_line() < LINE_DIST_TOLERANCE and \
            get_dist_to_goal() + MIN_DIST_IMPROVEMENT < DIST_QH:

        OBSTACLE = False
        vel.linear.x = 0
        vel.angular.z = 0
        return vel

    if is_blocked():  # Turning left
        vel.angular.z = kt
        return vel
    else:
        vel.linear.x = kx

        if can_turn_right():  # Turning right
            right_distance = get_average_dist_right()
            vel.linear.x = kx / right_distance
            vel.angular.z = - kt * right_distance
            return vel

        return vel


def bug2():
    '''
        Control the robot towards the goal position, using the Bug 2 algorithm.

        @return: (Twist) Velocity object.
    '''

    if OBSTACLE:
        return outline_obstacle()

    return follow_line()


def run():
    '''
        Main program.
    '''

    rospy.init_node('bug2', anonymous=True)
    set_params()

    # Messages
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=QUEUE_SIZE)
    rospy.Subscriber('/odom', Odometry, OdomCallback)
    rospy.Subscriber('/base_scan', LaserScan, LaserCallback)

    rate = rospy.Rate(RATE)
    cmd_vel = Twist()

    # Start algorithm execution.
    start_time = time()
    while not rospy.is_shutdown():
        if laserMsg is None:
            rate.sleep()
            continue

        if odomMsg:
            if get_dist_to_goal() < GOAL_TOLERANCE:
                print 'Goal achieved'
                end_time = time()
                break

            if line_a is None:
                set_goal_line()

            cmd_vel = bug2()

        pub.publish(cmd_vel)
        rate.sleep()

    print 'Time elapsed:', round(end_time - start_time, 2), 's'
    return


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
