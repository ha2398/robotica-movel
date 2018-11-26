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

from ga import GeneticAlgorithm
from robot import Robot
from sys import argv


RATE = 10
QUEUE_SIZE = 10


def get_args():
    '''
        Get command line arguments.
    '''

    args = {
        'rewards': argv[1],
        'map': argv[2],
        'max_cost': float(argv[3]),
        'start_x': float(argv[4]),
        'start_y': float(argv[5]),
        'end_x': float(argv[6]),
        'end_y': float(argv[7])
    }

    return args


def run():
    '''
        Main program.
    '''

    args = get_args()
    rospy.init_node('pf', anonymous=True)

    # Load input
    sampling_points = np.genfromtxt(
        args['rewards'], delimiter=' ', dtype=float)
    environment_map = np.genfromtxt(args['map'], delimiter=' ', dtype=int)

    # Run the GA heuristic to get robot path.
    ga_op = GeneticAlgorithm(
        sampling_points, environment_map, args['max_cost'])

    path = ga_op.run((args['start_x'], args['start_y']),
                     (args['end_x'], args['end_y']))

    robot = Robot(RATE, QUEUE_SIZE)

    while not robot.is_ready():
        pass

    goal = None
    # Control robot through the path.
    while not rospy.is_shutdown() and len(path) > 0:
        if goal is None or not robot.bug2(*goal):
            goal = path.pop(0)

    return


def main():
    try:
        run()
    except rospy.ROSInterruptException:
        pass


main()
