#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

laserMsg = None

def LaserCallback(msg):
	global laserMsg
	laserMsg = msg

def run():
	rospy.init_node('move_example', anonymous=False)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	rospy.Subscriber('/base_scan_1', LaserScan, LaserCallback)

	rate = rospy.Rate(5) # 5 hz
	print "Running..."
	global laserMsg

	cmd_vel = Twist()
	cmd_vel.linear.x = 1.0

	while not rospy.is_shutdown():
		if laserMsg == None:
			rate.sleep()
			continue

		if laserMsg.ranges[len(laserMsg.ranges)/2] > 4:
			cmd_vel.linear.x = 1.0
			cmd_vel.angular.z = 0.0
		elif laserMsg.ranges[len(laserMsg.ranges)*1/4] < 4:
			cmd_vel.linear.x = 0.0
			cmd_vel.angular.z = 3.0
		elif laserMsg.ranges[len(laserMsg.ranges)*3/4] < 4:
			cmd_vel.linear.x = 0.0
			cmd_vel.angular.z = -3.0
		pub.publish(cmd_vel)
		rate.sleep()

if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
			pass