#!/usr/bin/env python2

'''
@author: Hugo Sousa

Mobile Robotics
Final Project
Orienteering Problem

grid.py: Map<->Grid utilities.
'''

import numpy as np

from scipy import ndimage


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

    return (j - (width - 1) / 2., (height - 1) / 2. - i)


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


def get_neighbours(array, index):
        '''
            Get neighbour cells of the cell with given index in an array.

            @array: (numpy array) Array.
            @index: (int, int) Index of cell.

            @return: (numpy array) Array with the indices of neighbour cells.
        '''

        matrix = np.array(array)
        indices = tuple(np.transpose(np.atleast_2d(index)))
        arr_shape = np.shape(matrix)
        dist = np.ones(arr_shape)
        dist[indices] = 0
        dist = ndimage.distance_transform_cdt(dist, metric='chessboard')
        nb_indices = np.transpose(np.nonzero(dist == 1))
        return [tuple(x) for x in nb_indices]
