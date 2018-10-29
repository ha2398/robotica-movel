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
    OCC_THRESHOLD = 0.6
    INITIAL_P = 0.5

    def __init__(self, height, width, resolution, max_laser_range, num_ranges):
        '''
            @height: (int) Number of cells that form the grid's height.
            @width: (int) Number of cells that form the grid's width.
            @resolution: (float) Size of the cells side.
            @max_laser_range: (float) Max distance of laser ranges.
            @num_ranges: (int) Number of laser beams in robot's sensor.
        '''

        self.height = height
        self.width = width

        self.cells = np.full((height, width),
                             self.log_odds_ratio(self.INITIAL_P))
        self.resolution = resolution

        # Thickness of obstacles
        self.alpha = 1.0
        # Width of each laser beam.
        self.beta = (max_laser_range * np.pi) / num_ranges

        self.l_occ = self.log_odds_ratio(self.OCC_THRESHOLD)
        self.l_free = self.log_odds_ratio(1 - self.OCC_THRESHOLD)

    def log_odds_ratio(self, p):
        '''
            Calculate the log odds ratio for a give probability.

            @p: (float) Probability.

            @return: (float) Log odds ratio for @p.
        '''

        return np.log(p / (1 - p))

    def get_prob_from_log_odds(self, l):
        '''
            Retrieve probability from log odds ratio.

            @l: (float) Log odds ratio.

            @return: (float) Probability.
        '''

        return 1 - (1 / (1 + np.exp(l)))

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

    def center_of_mass_from_index(self, i, j):
        '''
            Get the coordinates of the center of mass of the cell with the
            given index.

            @i: (int) Grid row.
            @j: (int) Grid column.

            @return: (float, float) Coordintes of the cell's center of mass.
        '''

        x = self.resolution * (j - (self.height + 1) / 2.)
        y = self.resolution * (i - (self.width + 1) / 2.)

        return x, y

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

        msg.data = [int(self.get_prob_from_log_odds(
            x) * self.MAX_GRAY_VALUE) for x in self.cells.flat]

        return msg
