#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


odomMsg = None
target = (7, 8.123, 0)
kx, ky, kz = 1, 1, 1


def LaserCallback(msg):
    global laserMsg
    laserMsg = msg


def OdomCallback(msg):
    global odomMsg
    odomMsg = msg


def run():
    global laserMsg
    rospy.init_node('move_example', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, OdomCallback)
    rate = rospy.Rate(5)
    cmd_vel = Twist()

    while not rospy.is_shutdown():
        if odomMsg:
            x = odomMsg.pose.pose.position.x
            y = odomMsg.pose.pose.position.y
            z = odomMsg.pose.pose.position.z

            ex = target[0] - x
            ey = target[1] - y
            ez = target[2] - z

            print 'Position:\n', odomMsg.pose.pose.position, '\n'
            print 'Error:\n', 'ex:', ex, 'ey:', ey, 'ez:', ez

            cmd_vel.linear.x = kx * ex
            cmd_vel.linear.y = ky * ey

        pub.publish(cmd_vel)
        rate.sleep()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
