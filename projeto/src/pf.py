#!/usr/bin/env python2

'''
@author: Hugo Sousa

Mobile Robotics
Final Project
Orienteering Problem
'''

import rospy

from sys import argv


RATE = 10
QUEUE_SIZE = 10


def get_args():
    '''
        Get command line arguments.
    '''

    args = {
    }

    return args


def run():
    '''
        Main program.
    '''

    args = get_args()
    rospy.init_node('pf', anonymous=True)
    return


def main():
    try:
        run()
    except rospy.ROSInterruptException:
        pass


main()
