#!/usr/bin/env python2

'''
@author: Hugo Sousa

Mobile Robotics
Final Project
Orienteering Problem

pf.py: Main program.
'''

import numpy as np
import rospy

from nav_msgs.msg import MapMetaData, OccupancyGrid
from sys import argv


RATE = 10
QUEUE_SIZE = 10


def get_args():
    '''
        Get command line arguments.
    '''

    args = {
        'costmap': argv[1]
    }

    return args


def run():
    '''
        Main program.
    '''

    args = get_args()
    print(args['costmap'])
    costmap = np.genfromtxt(args['costmap'], delimiter=' ')

    rospy.init_node('pf', anonymous=True)
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=QUEUE_SIZE)
    rate = rospy.Rate(RATE)

    while True:
        msg = OccupancyGrid()
        data = costmap.ravel()
        msg.data = data
        msg.info = MapMetaData()
        msg.info.height = costmap.shape[0]
        msg.info.width = costmap.shape[1]
        msg.info.resolution = 1
        map_pub.publish(msg)
        rate.sleep()

    return


def main():
    try:
        run()
    except rospy.ROSInterruptException:
        pass


main()
