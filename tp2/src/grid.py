#!/usr/bin/env python2

'''
@author: Hugo Sousa

Class Grid models the robot environment as a grid of cells.
'''

from nav_msgs.msg import OccupancyGrid
from scipy import ndimage

import numpy as np
import rospy


class Grid:

    # Constants
    OUTPUT_NAME = 'map.pgm'
    PGM_MAGIC_NUM = 'P2'
    MAX_GRAY_VALUE = 100
    OCC_THRESHOLD = 0.7
    INITIAL_P = 0.5
    ALPHA = 0.5  # Thickness of obstacles

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

            @return: (float, float) Coordinates of the cell's center of mass.
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

    def get_neighbours(self, array, index):
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

    def get_random_frontier_cell(self):
        '''
            Get random frontier cell coordinates.

            @return: (float, float) Closest frontier cell coordinates.
        '''

        probs = np.flipud(self.get_prob_from_log_odds(self.cells))
        visited = np.full((self.height, self.width), False)

        # Create random search starting point.
        rand_cell_i = np.random.randint(0, self.height)
        rand_cell_j = np.random.randint(0, self.width)
        random_starting_point = self.center_of_mass_from_index(
            rand_cell_i, rand_cell_j)

        queue = [self.position_to_index(*random_starting_point)]
        occ_prob = self.OCC_THRESHOLD
        free_prob = 1 - occ_prob

        while len(queue) > 0:  # BFS
            cur_cell = queue.pop(0)

            if visited[cur_cell]:
                continue

            visited[cur_cell] = True

            neighbours = self.get_neighbours(probs, cur_cell)
            queue += neighbours

            if probs[cur_cell] <= free_prob:  # Free cell
                while len(neighbours) > 0:
                    n = neighbours.pop(0)
                    p = probs[n]

                    if p > free_prob and p < occ_prob:  # Unknown cell
                        return self.center_of_mass_from_index(*cur_cell)

        return None

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
        msg.info.origin.orientation.x = 0
        msg.info.origin.orientation.y = 0
        msg.info.origin.orientation.z = 0
        msg.info.origin.orientation.w = 1
        msg.header.stamp = rospy.Time.now()

        cells = np.flipud(self.cells)
        msg.data = [int(self.get_prob_from_log_odds(
            x) * self.MAX_GRAY_VALUE) for x in cells.flat[::-1]]

        return msg

    def dump_pgm(self):
        '''
            Dump the grid probabilities to a PGM image file.
        '''

        header_str = '{}\n{} {}\n{}'.format(
            self.PGM_MAGIC_NUM, self.width, self.height, self.MAX_GRAY_VALUE)

        cells = np.rot90(self.cells)
        cells = np.vectorize(lambda x: int(
            (1 - self.get_prob_from_log_odds(x)) * self.MAX_GRAY_VALUE))(cells)

        np.savetxt(self.OUTPUT_NAME, cells, delimiter=' ',
                   header=header_str, comments='', fmt='%d')
