#!/usr/bin/env python2

'''
@author: Hugo Sousa

Mobile Robotics - TP2
'''

import rospy

from robot import Robot
from sys import argv
from time import sleep


RATE = 10
QUEUE_SIZE = 10
EXIT_TIMER = 5


def get_args():
    '''
        Get command line arguments.
    '''

    args = {
        'height': int(argv[1]),
        'width': int(argv[2]),
        'resolution': float(argv[3]),
        'frontiers': int(argv[4])
    }

    return args


def run():
    '''
        Main program.
    '''

    args = get_args()
    rospy.init_node('tp2', anonymous=True)
    robot = Robot(RATE, QUEUE_SIZE)

    while not robot.is_ready():
        pass

    robot.occupancy_grid(args['height'], args['width'], args['resolution'],
                         args['frontiers'])
    print 'Done mapping. Exiting in', EXIT_TIMER, 's.'
    sleep(EXIT_TIMER)
    return


def main():
    try:
        run()
    except rospy.ROSInterruptException:
        pass


main()
