#!/usr/bin/env python2

import rospy

from geometry_msgs.msg import Twist
from math import atan2, cos, degrees, radians, sin
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sys import argv
from time import time


RATE = 20
QUEUE_SIZE = 10

# Speed constants
kt = 1
k_att, k_rep = 10, 0.1

p_0 = 4

# Messages
laserMsg = None
odomMsg = None

GOAL_TOLERANCE = 0.1
GOAL = None
theta = None


def LaserCallback(msg):
    global laserMsg
    laserMsg = msg


def OdomCallback(msg):
    global odomMsg
    odomMsg = msg


def yawFromQuaternion(orientation):
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    return atan2((2.0 * (w * z + x * y)), (1.0 - 2.0 * (y * y + z * z)))


def get_error():
    ex = GOAL[0] - odomMsg.pose.pose.position.x
    ey = GOAL[1] - odomMsg.pose.pose.position.y

    return ex, ey


def get_dist_to_goal():
    x, y = get_robot_coordinates()
    return euclidean_distance((x, y), GOAL)


def get_robot_coordinates():
    x = odomMsg.pose.pose.position.x
    y = odomMsg.pose.pose.position.y

    return x, y


def euclidean_distance(p1, p2):
    return sum([(p1_i - p2_i) ** 2 for (p1_i, p2_i) in zip(p1, p2)]) ** 0.5


def get_f_att():
    x, y = get_robot_coordinates()
    magnitude = k_att * get_dist_to_goal()
    angle = atan2(GOAL[1] - y, GOAL[0] - x)
    return [cos(angle) * magnitude, sin(angle) * magnitude]


def get_f_rep():
    f_rep = [0, 0]
    yaw = yawFromQuaternion(odomMsg.pose.pose.orientation)

    for i, d in enumerate(laserMsg.ranges[:-1]):  # 360 degrees
        if d > p_0:
            continue

        angle = radians((i / 2)) + yaw + radians(90)
        magnitude = k_rep * ((1 / d) - (1 / p_0)) * (1 / (d ** 2))
        f_i = [cos(angle) * magnitude, sin(angle) * magnitude]
        f_rep = sum_vectors(f_rep, f_i)

    return f_rep


def sum_vectors(v1, v2):
    return [v1_i + v2_i for v1_i, v2_i in zip(v1, v2)]


def get_force_vector():
    f_att = get_f_att()
    f_rep = get_f_rep()

    m_att = sum(v_i ** 2 for v_i in f_att) ** .5
    m_rep = sum(v_i ** 2 for v_i in f_rep) ** .5

    print 'ATTRACTION\t', 'theta:', degrees(atan2(f_att[1], f_att[0])), 'magnitude:', m_att
    print 'REPULSION\t', 'theta:', degrees(atan2(f_rep[1], f_rep[0])), 'magnitude:', m_rep

    return sum_vectors(f_att, f_rep)


def get_norm(v):
    return sum(v_i ** 2 for v_i in v) ** 0.5


def set_params():
    global GOAL

    GOAL = float(argv[1]), float(argv[2])


def potential_fields():
    f = get_force_vector()
    fx, fy = f

    theta = atan2(fy, fx)

    m = get_norm(f)
    print 'RESULTANT\t', 'theta:', degrees(theta), 'magnitude:', m
    print

    vel = Twist()
    yaw = yawFromQuaternion(odomMsg.pose.pose.orientation)

    x = fx * cos(yaw)
    y = fy * sin(yaw)
    vel.linear.x = x * cos(yaw) + y * sin(yaw)
    vel.linear.y = y * cos(yaw) - x * sin(yaw)
    vel.angular.z = kt * (theta - yaw)

    return vel


def run():
    global laserMsg

    rospy.init_node('potential_fields', anonymous=True)
    set_params()

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=QUEUE_SIZE)
    rospy.Subscriber('/odom', Odometry, OdomCallback)
    rospy.Subscriber('/base_scan', LaserScan, LaserCallback)

    rate = rospy.Rate(RATE)
    cmd_vel = Twist()

    start_time = time()
    while not rospy.is_shutdown():
        if laserMsg is None:
            rate.sleep()
            continue

        if odomMsg:
            x, y = get_robot_coordinates()
            dist_to_goal = euclidean_distance((x, y), GOAL)

            if dist_to_goal < GOAL_TOLERANCE:
                print 'Goal achieved'
                end_time = time()
                break

            cmd_vel = potential_fields()
            yaw = yawFromQuaternion(odomMsg.pose.pose.orientation)
            print 'YAW:', degrees(yaw)

            print 'VELOCITIES'
            print 'x:', cmd_vel.linear.x
            print 'y:', cmd_vel.linear.y
            print 'w:', cmd_vel.angular.z

        # pub.publish(cmd_vel)
        rate.sleep()

    print 'Time elapsed:', round(end_time - start_time, 2), 's'
    return


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
