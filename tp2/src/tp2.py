#!/usr/bin/env python2

'''
@author: Hugo Sousa

Mobile Robotics - TP2
'''

import rospy
from robot import Robot


def run():
    '''
        Main program.
    '''

    rospy.init_node('tp2', anonymous=True)
    robot = Robot()

    goal = (5, 6)

    while not rospy.is_shutdown():
        if not robot.bug2(*goal):
            break

    print 'Done'

    return


def main():
    try:
        run()
    except rospy.ROSInterruptException:
        pass


print('hi')
main()
