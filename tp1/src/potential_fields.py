#!/usr/bin/env python2

import rospy

from geometry_msgs.msg import Twist
from math import atan2, cos, radians, sin
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sys import argv
from time import time

RATE = 20
QUEUE_SIZE = 10

# Speed constants
kt = 5
k_att, k_rep = 10, 5

# Messages
laserMsg = None
odomMsg = None

# Tolerance constants.
GOAL_TOLERANCE = 0.1
GOAL = None
theta = None
p_0 = 2


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

    ex = GOAL[0] - odomMsg.pose.pose.position.x
    ey = GOAL[1] - odomMsg.pose.pose.position.y

    return ex, ey


def get_dist_to_goal():
    '''
        Calculate the robot's distance to the goal position.

        @return: (float) Robot's distance to goal.
    '''

    x, y = get_robot_coordinates()
    return euclidean_distance((x, y), GOAL)


def get_robot_coordinates():
    '''
        Get current robot coordinates in space.

        @return: (float, float) Tuple with x and y coordinates of the robot.
    '''

    x = odomMsg.pose.pose.position.x
    y = odomMsg.pose.pose.position.y

    return x, y


def euclidean_distance(p1, p2):
    '''
        Calculate the Euclidean distance between two 2D points.

        @p1: (float, float) Point 1 coordinates.
        @p2: (float, float) Point 2 coordinates.

        @return: (float) Euclidean distance between points 1 and 2.
    '''

    return sum([(p1_i - p2_i) ** 2 for (p1_i, p2_i) in zip(p1, p2)]) ** 0.5


def get_f_att():
    '''
        Calculate the attraction force towards the goal position.

        @return: (float, float) Attraction force vector.
    '''

    x, y = get_robot_coordinates()
    magnitude = k_att * get_dist_to_goal()
    angle = atan2(GOAL[1] - y, GOAL[0] - x)
    return [cos(angle) * magnitude, sin(angle) * magnitude]


def get_f_rep():
    '''
        Calculate the repulsion force made by obstacles.

        @return: (float, float) Repulsion force vector.
    '''

    f_rep = [0, 0]
    yaw = yawFromQuaternion(odomMsg.pose.pose.orientation)

    for i, d in enumerate(laserMsg.ranges[:-1]):  # 360 degrees
        if d > p_0:
            continue

        angle = radians((i / 2)) + yaw
        magnitude = k_rep * ((1 / d) - (1 / p_0)) * (1 / (d ** 2))
        f_i = [cos(angle) * magnitude, sin(angle) * magnitude]
        f_rep = sum_vectors(f_rep, f_i)

    return f_rep


def sum_vectors(v1, v2):
    '''
        Sum two vectors.

        @v1: (float, float) Vector 1 coordinates.
        @v2: (float, float) Vector 2 coordinates.

        @return: (float, float) Resultant vector coordinates.
    '''

    return [v1_i + v2_i for v1_i, v2_i in zip(v1, v2)]


def get_force_vector():
    '''
        Calculate the resultant force vector acting on the robot.

        @return: (float, float) Resultant vector coordinates.
    '''

    f_att = get_f_att()
    f_rep = get_f_rep()
    return sum_vectors(f_att, f_rep)


def get_norm(v):
    '''
        Get the norm of a vector.

        @v: (float, float) Vector coordinates.

        @return: (float) Vector norm.
    '''

    return sum(v_i ** 2 for v_i in v) ** 0.5


def set_params():
    '''
        Set the input parameters.
    '''

    global GOAL

    GOAL = float(argv[1]), float(argv[2])
    return


def potential_fields():
    '''
        Control the robot towards the goal position, using the Potential
        Fields algorithm.

        @return: (Twist) Velocity object.
    '''

    f = get_force_vector()
    fx, fy = f

    theta = atan2(fy, fx)

    vel = Twist()
    yaw = yawFromQuaternion(odomMsg.pose.pose.orientation)

    vel.linear.x = fx * cos(yaw) + fy * sin(yaw)
    vel.linear.y = fy * cos(yaw) - fx * sin(yaw)
    vel.angular.z = kt * (theta - yaw)

    return vel


def run():
    '''
        Main program.
    '''

    rospy.init_node('potential_fields', anonymous=True)
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

            cmd_vel = potential_fields()

        pub.publish(cmd_vel)
        rate.sleep()

    print 'Time elapsed:', round(end_time - start_time, 2), 's'
    return


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
