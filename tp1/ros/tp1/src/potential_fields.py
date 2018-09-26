#!/usr/bin/env python2

import rospy

from geometry_msgs.msg import Twist
from math import sin, cos, atan2, pi
from nav_msgs.msg import Odometry


odomMsg = None
target = (2, 4, pi / 2)
kx, ky, kt, kp = 1, 1, 1, 1
tolerance = 0.001
tolerance_a = 0.01

goal = False


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
    ex = target[0] - odomMsg.pose.pose.position.x
    ey = target[1] - odomMsg.pose.pose.position.y
    et = target[2] - yawFromQuaternion(odomMsg.pose.pose.orientation)

    return ex, ey, et


def differential_controller():
    global goal

    vel = Twist()
    ex, ey, et = get_error()
    theta = yawFromQuaternion(odomMsg.pose.pose.orientation)

    vel.linear.x = kp * ex * cos(theta)
    vel.linear.y = kp * ey * sin(theta)
    vel.angular.z = kt * (atan2(ey, ex) - theta)

    if ex < tolerance and ey < tolerance and et < tolerance_a:
        vel.linear.x = 0
        vel.linear.y = 0
        vel.angular.z = 0

    return vel


def run():
    global laserMsg
    rospy.init_node('move_example', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, OdomCallback)
    rate = rospy.Rate(5)
    cmd_vel = Twist()

    while not rospy.is_shutdown():
        if odomMsg and not goal:
            cmd_vel = differential_controller()

        pub.publish(cmd_vel)
        rate.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
