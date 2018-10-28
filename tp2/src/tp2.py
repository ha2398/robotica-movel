#!/usr/bin/env python2

'''
@author: Hugo Sousa

Mobile Robotics - TP2
'''

import rospy

from robot import Robot
from sys import argv


RATE = 10
QUEUE_SIZE = 10


def get_args():
    '''
        Get command line arguments.
    '''

    args = {
        'height': int(argv[1]),
        'width': int(argv[2]),
        'resolution': float(argv[3]),
        'initial_p': float(argv[4])
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

    robot.occupancy_grid(args['height'], args['width'],
                         args['initial_p'], args['resolution'])
    return


def main():
    try:
        run()
    except rospy.ROSInterruptException:
        pass


main()
