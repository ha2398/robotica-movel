#!/usr/bin/env python2

'''
@author: Hugo Sousa

Class Grid models the robot environment as a grid of cells.
'''

import numpy as np


class Grid:

    # Constants
    OUTPUT_NAME = 'map.pgm'
    PGM_MAGIC_NUM = 'P2'
    MAX_GRAY_VALUE = 100

    def __init__(self, height, width, initial_p, origin, resolution):
        '''
            @height: (int) Number of cells that form the grid's height.
            @width: (int) Number of cells that form the grid's width.
            @initial_p: (float) Initial blocked probability for each cell.
            @origin: (float, float) Origin of the map in the coordinate.
            @resolution: (float) Size of the cells side.
        '''

        self.height = height
        self.width = width
        self.origin = origin
        self.cells = np.full((height, width), initial_p)
        self.resolution = resolution

    def position_to_index(self, x, y):
        '''
            Get the cell index of a given position in the world.

            @x: (float) x coordinate of position in the world frame.
            @y: (float) y coordinate of position in the world frame.

            @return: (int, int) Index of the corresponding grid cell of the
                given position.
        '''

        # Convert coordinates to Map frame.
        x, y = x - self.origin[0], y - self.origin[1]

        # Get grid cell index.
        i = int(y // self.resolution + self.width // 2)
        j = int(x // self.resolution + self.width // 2)

        return (i, j)

    def dump_pgm(self):
        '''
            Dump the grid probabilities to a PGM image file.
        '''

        with open(self.OUTPUT_NAME, 'w') as map_file:
            # Magic number
            map_file.write('{}\n'.format(self.PGM_MAGIC_NUM))
            map_file.write('{} {}\n'.format(self.width, self.height))
            map_file.write('{}\n'.format(self.MAX_GRAY_VALUE))

            np.flipud(self.cells)

            for h in range(self.height):
                for w in range(self.width):
                    value = self.cells[h, w]
                    value = int(round(value, 2) * self.MAX_GRAY_VALUE)
                    map_file.write('{} '.format(value))

                map_file.write('\n')

            np.flipud(self.cells)
