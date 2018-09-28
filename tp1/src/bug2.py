#!/usr/bin/env python2

import rospy

from geometry_msgs.msg import Twist
from math import atan2
from nav_msgs.msg import Odometry


odomMsg = None
target = (2, 4)
kx, ky, kt = 1, 1, 1
theta = None
set_front = False
angle_tolerance = 0.01


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
    global theta

    x = odomMsg.pose.pose.position.x
    y = odomMsg.pose.pose.position.y
    yaw = yawFromQuaternion(odomMsg.pose.pose.orientation)

    ex = target[0] - x
    ey = target[1] - y
    theta = atan2(ey, ex)
    et = theta - yaw

    return ex, et


def holonomic_controller():
    global set_front

    vel = Twist()
    ex, et = get_error()

    if not set_front:
        if abs(et) < angle_tolerance:
            set_front = True
        else:
            vel.linear.x = 0
            vel.angular.z = kt * et
    else:
        vel.linear.x = kx * ex

    return vel


def run():
    global laserMsg
    rospy.init_node('bug2', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, OdomCallback)
    rate = rospy.Rate(1000)
    cmd_vel = Twist()

    while not rospy.is_shutdown():
        if odomMsg:
            cmd_vel = holonomic_controller()

        pub.publish(cmd_vel)
        rate.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
