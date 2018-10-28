#!/usr/bin/env python2

'''
@author: Hugo Sousa

Class Grid models the robot environment as a grid of cells.
'''

from nav_msgs.msg import OccupancyGrid

import numpy as np
import rospy


class Grid:

    # Constants
    OUTPUT_NAME = 'map.pgm'
    PGM_MAGIC_NUM = 'P2'
    MAX_GRAY_VALUE = 100
    INITIAL_P = 0.5

    def __init__(self, height, width, resolution):
        '''
            @height: (int) Number of cells that form the grid's height.
            @width: (int) Number of cells that form the grid's width.
            @resolution: (float) Size of the cells side.
        '''

        self.height = height
        self.width = width

        # Give the robot space to explore in all directions.
        self.cells = np.full((height, width), self.INITIAL_P)
        self.resolution = resolution

    def position_to_index(self, x, y):
        '''
            Get the cell index of a given position in the world.

            @x: (float) x coordinate of position in the world frame.
            @y: (float) y coordinate of position in the world frame.

            @return: (int, int) Index of the corresponding grid cell of the
                given position.
        '''

        # Get grid cell index.
        i = int(y / self.resolution + self.width / 2)
        j = int(x / self.resolution + self.height / 2)

        return (i, j)

    def is_valid_index(self, i, j):
        '''
            Check if a given index is valid for the grid.

            @i: (int) Grid row.
            @j: (int) Grid column.

            @return: (bool) True iff the index is valid for the grid.
        '''

        return i >= 0 and j >= 0 and i < self.height and j < self.width

    def dump_pgm(self):
        '''
            Dump the grid probabilities to a PGM image file.
        '''

        with open(self.OUTPUT_NAME, 'w') as map_file:
            # Magic number
            map_file.write('{}\n'.format(self.PGM_MAGIC_NUM))
            map_file.write('{} {}\n'.format(self.width, self.height))
            map_file.write('{}\n'.format(self.MAX_GRAY_VALUE))

            cells = np.flipud(self.cells)

            for h in range(self.height):
                for w in range(self.width):
                    value = cells[h, w]
                    value = int(round(1 - value, 2) * self.MAX_GRAY_VALUE)
                    map_file.write('{} '.format(value))

                map_file.write('\n')

    def get_occupancy_msg(self):
        '''
            Create an Occupancy Grid message from grid data.

            @return: (OccupancyGrid) Occupancy Grid message.
        '''

        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = - self.width // 2 * self.resolution
        msg.info.origin.position.y = - self.height // 2 * self.resolution
        msg.header.stamp = rospy.Time.now()

        msg.data = [int(x * self.MAX_GRAY_VALUE) for x in self.cells.flat]

        return msg
