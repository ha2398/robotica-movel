#!/usr/bin/env python2

'''
@author: Hugo Sousa

Mobile Robotics
Final Project
Orienteering Problem

pf.py: Main program.
'''

import matplotlib.cm as cm
import matplotlib.pyplot as plt
import numpy as np
import rospy

from grid import *
from robot import Robot
from sys import argv

np.random.seed(0)

from ga import GeneticAlgorithm

RATE = 10
QUEUE_SIZE = 10


def print_path(height, width, rewards, path, cost, reward, max_cost):
    '''
        Plot a figure with the rewards in the map and the path found.

        @height: (int) Height of map.
        @width: (int) Width of map.
        @path: ((int, int) list) Path found.
        @cost: (float) Path cost.
        @reward: (float) Total path reward.
        @max_cost: (float) Max cost of OP instance.
    '''

    dim = rewards.shape
    markersize = (8 * 6 * 2625) / (width * height)

    x, y, colormap = [], [], []
    for i in range(dim[0]):
        for j in range(dim[1]):
            coord = index_to_center_of_mass(i, j, rewards)
            x.append(coord[0])
            y.append(coord[1])
            colormap.append(rewards[i, j])

    plt.figure(figsize=(10, 8), dpi=200)
    plt.rc('font', size=16)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.axhline(y=0, color='black', linewidth='1')
    plt.axvline(x=0, color='black', linewidth='1')
    sc = plt.scatter(x, y, c=colormap, cmap=cm.jet, marker='o', s=markersize)
    cb = plt.colorbar(sc)
    cb.set_label('Recompensas no mapa')
    plt.title('Limite de custo: {}\nCusto do caminho: {}\nRecompensa do caminho: {}'.format(
        max_cost, round(cost, 2), round(reward, 2)))

    path = list(zip(path, path[1:]))
    for line in path:
        x, y = list(zip(line[0], line[1]))
        plt.plot(x, y, linewidth=5, color='black', marker='x', linestyle='--',
                 markerfacecolor='white', markerfacecoloralt='white',
                 markeredgecolor='white')

    plt.savefig('heatmap{}x{}_maxc{}.png'.format(dim[0], dim[1], max_cost))


def get_args():
    '''
        Get command line arguments.
    '''

    args = {
        'rewards': argv[1],
        'max_cost': float(argv[2]),
        'start_x': float(argv[3]),
        'start_y': float(argv[4]),
        'end_x': float(argv[5]),
        'end_y': float(argv[6]),
        'gen': int(argv[7]),
        'pop': int(argv[8]),
        'navigate': int(argv[9]),
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

    print 'Starting Genetic Algorithm...'
    # Run the GA heuristic to get robot path.
    ga_op = GeneticAlgorithm(sampling_points, args['max_cost'],
                             args['gen'], args['pop'])

    path, cost, rewards = ga_op.run((args['start_x'], args['start_y']),
                                    (args['end_x'], args['end_y']),)
    print 'Done.'

    dim = sampling_points.shape
    filename = 'path_{}x{}_maxc{}.txt'.format(dim[0], dim[1], args['max_cost'])
    with open(filename, 'w') as output_file:
        output_file.write('{}\n'.format(args['max_cost']))
        output_file.write('{}\n'.format(round(cost, 2)))
        output_file.write('{}\n'.format(round(rewards, 2)))

        for coord in path:
            output_file.write('{}\n'.format(coord))

    print_path(dim[0], dim[1], sampling_points, path, cost,
               rewards, args['max_cost'])

    if args['navigate'] == 0:
        return

    robot = Robot(RATE, QUEUE_SIZE)

    while not robot.is_ready():
        pass

    print 'Starting navigation through path:', path
    goal = None
    # Control robot through the path.
    while not rospy.is_shutdown() and len(path) > 0:
        if goal is None or not robot.bug2(goal[0], goal[1]):
            goal = path.pop(0)

    print 'Done.'

    return


def main():
    try:
        run()
    except rospy.ROSInterruptException:
        pass


main()
