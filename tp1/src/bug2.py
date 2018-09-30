#!/usr/bin/env python2

import rospy

from geometry_msgs.msg import Twist
from math import atan2, degrees
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sys import argv

RATE = 20
QUEUE_SIZE = 10

# Speed constants
kx, ky, kt = 1, 1, 2

# Messages
laserMsg = None
odomMsg = None

TARGET = None
theta = None
SET_FRONT = False
ANGLE_TOLERANCE = 0.005

line_a, line_b, line_c = None, None, None

MIN_OBJ_DIST = 0.6
LINE_DIST_TOLERANCE = 0.1
GOAL_TOLERANCE = 0.1
MIN_DIST_IMPROVEMENT = 0.5

OBSTACLE = False
DIST_QH = None

# Ranges
FRONT_LEFT = 240
FRONT_RIGHT = 120
RIGHT_START = 150

GOAL_ACHIEVED = False


def LaserCallback(msg):
    global laserMsg
    laserMsg = msg


def OdomCallback(msg):
    global odomMsg
    odomMsg = msg


def get_robot_coordinates():
    x = odomMsg.pose.pose.position.x
    y = odomMsg.pose.pose.position.y

    return x, y


def yawFromQuaternion(orientation):
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    return atan2((2.0 * (w * z + x * y)), (1.0 - 2.0 * (y * y + z * z)))


def get_error():
    global theta

    x, y = get_robot_coordinates()
    yaw = yawFromQuaternion(odomMsg.pose.pose.orientation)

    ex = euclidean_distance((x, y), TARGET)
    theta = atan2(TARGET[1] - y, TARGET[0] - x)
    et = theta - yaw

    return ex, et


def set_params():
    global TARGET

    TARGET = float(argv[1]), float(argv[2])


def set_goal_line():
    global line_a, line_b, line_c

    x_a, y_a = get_robot_coordinates()
    x_b, y_b = TARGET

    m = (y_b - y_a) / (x_b - x_a)
    b = y_b - m * x_b

    line_a = - m
    line_b = 1
    line_c = - b


def get_distance_to_line():
    x, y = get_robot_coordinates()

    return abs(line_a * x + line_b * y + line_c) / \
        ((line_a ** 2 + line_b ** 2) ** 0.5)


def euclidean_distance(p1, p2):
    return sum([(p1_i - p2_i) ** 2 for (p1_i, p2_i) in zip(p1, p2)]) ** 0.5


def follow_line():
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
            x, y = get_robot_coordinates()
            dist_to_target = euclidean_distance((x, y), TARGET)
            OBSTACLE = True
            DIST_QH = dist_to_target
            SET_FRONT = False
            vel.linear.x = 0
            vel.angular.z = 0
            return vel

        vel.linear.x = kx * abs(ex)

    return vel


def is_blocked():
    ranges = [True if r <= MIN_OBJ_DIST else False
              for r in laserMsg.ranges[FRONT_RIGHT:FRONT_LEFT]]

    return any(ranges)


def get_average_dist_front():
    ranges = laserMsg.ranges[FRONT_RIGHT:FRONT_LEFT]

    return sum(ranges) / len(ranges)


def get_average_dist_right():
    ranges = laserMsg.ranges[:RIGHT_START]

    return sum(ranges) / len(ranges)


def can_turn_right():
    ranges = [True if r > MIN_OBJ_DIST else False
              for r in laserMsg.ranges[:RIGHT_START]]

    return all(ranges)


def outline_obstacle():
    global OBSTACLE

    vel = Twist()
    coord = get_robot_coordinates()

    # Check if robot can move to TARGET again.
    if get_distance_to_line() < LINE_DIST_TOLERANCE and \
            euclidean_distance(coord, TARGET) + MIN_DIST_IMPROVEMENT < DIST_QH:

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
    global DIST_QH, OBSTACLE, SET_FRONT

    if OBSTACLE:
        return outline_obstacle()

    return follow_line()


def run():
    global laserMsg

    rospy.init_node('bug2', anonymous=True)
    set_params()

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=QUEUE_SIZE)
    rospy.Subscriber('/odom', Odometry, OdomCallback)
    rospy.Subscriber('/base_scan', LaserScan, LaserCallback)

    rate = rospy.Rate(RATE)
    cmd_vel = Twist()

    while not rospy.is_shutdown():
        if laserMsg is None:
            rate.sleep()
            continue

        if odomMsg:
            x, y = get_robot_coordinates()
            dist_to_target = euclidean_distance((x, y), TARGET)

            if dist_to_target < GOAL_TOLERANCE:
                print('Goal achieved')
                return

            if line_a is None:
                set_goal_line()

            cmd_vel = bug2()

        pub.publish(cmd_vel)
        rate.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
