#!/usr/bin/env python2

'''
@author: Hugo Sousa

Mobile Robotics
Final Project
Orienteering Problem

grid.py: Map<->Grid utilities.
'''


def index_to_center_of_mass(i, j, array):
    '''
        Get the center of mass coordinate of a given grid cell in the world.

        @i: (int) Row index of cell.
        @j: (int) Column index of cell.
        @array: (2D numpy array) Array to which the cell belongs.

        @return: (float. float) Center of mass coordinate of the cell in the
            world.
    '''

    width = array.shape[1]
    height = array.shape[0]

    return (j - (width - 1) / 2, (height - 1) / 2 - i)


def position_to_index(x, y, array):
    '''
        Get the grid cell index of a position in the world.

        @x: (float) x coordinate of position.
        @y: (float) y coordinate of position.
        @array: (2D numpy array) Array to which the cell belongs.

        @return: (int, int) Grid cell index of the position in the world.
    '''

    width = array.shape[1]
    height = array.shape[0]

    return int((height - 1) / 2 - y), int(x + (width - 1) / 2)
