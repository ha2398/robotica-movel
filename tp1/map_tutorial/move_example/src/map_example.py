#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

mapMsg = None

def MapCallback(msg):
	global mapMsg
	mapMsg = msg

def run():
	rospy.init_node('map_example', anonymous=False)
	rospy.Subscriber('/map', OccupancyGrid, MapCallback)

	rate = rospy.Rate(5) # 5 hz

	print "[Map Example Started]"

	global mapMsg

	while not rospy.is_shutdown():
		if mapMsg == None:
			rate.sleep()
			continue
		mapa = np.asarray(mapMsg.data)
		mapa = mapa.reshape((mapMsg.info.height,mapMsg.info.width))
		print mapa
		rate.sleep()

if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
			pass