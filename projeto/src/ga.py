#!/usr/bin/env python2

'''
@author: Hugo Sousa

Mobile Robotics
Final Project
Orienteering Problem

ga.py: Genetic Algorithm methods.
'''


import numpy as np

from grid import *


class GeneticAlgorithm:

    def __init__(self, sampling_points, environment_map, max_cost):
        '''
            Initialize the OP GA instance.

            @sampling_points: (2D numpy array) Array that represents the
                sampling points in the environment and their rewards.
            @environment_map: (2D numpy array) Array that represents the
                environment map as a Occupancy Grid map. Each cell stores 0,
                in case it's free and 1, in case it's occupied by an obstacle.
            @max_cost: (float) Max path cost.
        '''

        self.sampling_points = sampling_points
        self.environment_map = environment_map
        self.max_cost = max_cost

    def run(self, start, finish, pop_size, max_gen):
        '''
            Run the genetic algorithm instance.

            @start: (int, int) Robot's path start point.
            @finish: (int, int) Robot's path end point.
            @pop_size: (int) Desired number of individuals to be generated.
            @max_gen: (int) Maximum number of generations to run the algorithm
                for.

            @return: ((int, int) list) Fittest chromosome.
        '''

        pass  # TODO

    def get_path_cost(self, path):
        '''
            Get the total cost of a path.

            @path: ((int, int) list) Path to get cost for.

            @return: (float) Path cost.
        '''

        cost = 0
        for start, end in list(zip(path, path[1:])):
            p1 = np.array(index_to_center_of_mass(start))
            p2 = np.array(index_to_center_of_mass(end))

            cost += np.linalg.norm(p2 - p1)

        return cost

    def initialize_population(self, pop_size, start, finish):
        '''
            Initialize the genetic algorithm population with randomly generated
            individuals.

            @pop_size: (int) Desired number of individuals to be generated.
            @start: (int, int) Robot's path start point.
            @finish: (int, int) Robot's path end point.

            @return: (((int, int) list) list) New random population.
        '''

        population = []
        done = False
        map_dim = self.environment_map.shape

        for _ in range(pop_size):
            chromosome = [start]

            # Retrieve available vertices.
            vertices = []
            for i in range(map_dim[0]):
                for j in range(map_dim[1]):
                    if self.environment_map[(i, j)] == 1:
                        vertices.append((i, j))

            while not done and len(vertices) != 0:
                v = np.random.choice(vertices)

                if self.get_path_cost(chromosome + [v]) <= self.max_cost:
                    chromosome.append(v)
                    vertices.remove(v)
                else:
                    done = True

            chromosome.append(finish)
            population.append(chromosome)

        return population

    def select_new_population(self):
        '''
            Select the new population for the next generation of individuals.
        '''

        pass  # TODO

    def crossover(self, parent1, parent2):
        '''
            Perform the crossover using two parents to generate two new
            children individuals.

            @parent1: ((int, int) list) First parent.
            @parent2: ((int, int) list) Second parent.

            @return:
                ((int, int) list) First child,
                ((int, int) list) Second child.
        '''

        # Check if the two parents share genes.
        if not set(parent1).isdisjoint(set(parent2)):  # Yes, they do.
            common_vertices = list(set(parent1).intersection(set(parent2)))

            # Randomly select a splitting point.
            random_common_vertex = np.random.choice(
                range(len(common_vertices)))
            v = common_vertices[random_common_vertex]

            index_v_1 = parent1.index(v)
            index_v_2 = parent2.index(v)

            tail_1 = parent1[index_v_1:]
            tail_2 = parent2[index_v_2:]

            # Swap tails
            child1 = [tuple(t) for t in parent1[:index_v_1] + tail_2]
            child2 = [tuple(t) for t in parent2[:index_v_2] + tail_1]

            return child1, child2
        else:  # No, they don't.
            return parent1, parent2

    def mutate(self):
        '''
            Mutate an individual's genes.
        '''

        pass  # TODO

    def evaluate_chromosome(self, chromosome):
        '''
            Calculate how fit an individual is, based on their genes.

            @chromosome: ((int, int) list) Chromosome to calculate fitness for.

            @return: (float) Chromosome fitness.
        '''

        rewards = 0

        for gene in chromosome:
            rewards += self.sampling_points(gene)

        return (rewards ** 3) / self.get_path_cost(chromosome)

    def get_fittest_chromosome(self, population):
        '''
            Get the fittest chromosome in a population.

            @population: (((int, int) list) list) Population of individuals.

            @return: ((int, int) list) Fittest chromosome in the population.
        '''

        by_fitness = sorted(
            population, key=self.evaluate_chromosome, reverse=True)

        return by_fitness[0]
