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

    # Fixed from literature
    POP_SIZE = 450
    GEN_NUMBER = 35
    TOURNAMENT_SIZE = 3
    CX_PROB = 0.6
    MUT_PROB = 0.8
    ELIT_PCT = 0.07

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

    def run(self, start, finish):
        '''
            Run the genetic algorithm instance.

            @start: (int, int) Robot's path start point.
            @finish: (int, int) Robot's path end point.

            @return: ((int, int) list) Fittest chromosome.
        '''

        population = self.initialize_population(start, finish)
        generation = 0

        while generation < self.GEN_NUMBER:
            population = self.select_new_population(population)

        return self.get_fittest_chromosome(population)

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

    def initialize_population(self, start, finish):
        '''
            Initialize the genetic algorithm population with randomly generated
            individuals.

            @start: (int, int) Robot's path start point.
            @finish: (int, int) Robot's path end point.

            @return: (((int, int) list) list) New random population.
        '''

        population = []
        done = False
        map_dim = self.environment_map.shape

        for _ in range(self.POP_SIZE):
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

    def tournament_selection(population):
        '''
            Perform a tournament selection on a population of individuals.

            @population: (((int, int) list) list) Population to perform the
                selection on.

            @return: ((int, int) list) Selected individual.
        '''

        indices = np.random.choice(range(len(population)),
                                   self.TOURNAMENT_SIZE, replace=False)

        individuals = [population[i] for i in indices]

        return self.get_fittest_chromosome(individuals)

    def get_random_parents(population):
        '''
            Select two individuals from a population to procriate.

            @population: (((int, int) list) list) Population to perform the
                selection on.

            @return:
                ((int, int) list) Parent 1.
                ((int, int) list) Parent 2.
        '''

        parent1 = self.tournament_selection(population)
        parent2 = self.tournament_selection(population)

        # Make sure the parents are two different individuals
        while parent1 == parent2:
            parent2 = self.tournament_selection(population)

        return parent1, parent2

    def select_new_population(self, current_population):
        '''
            Select the new population for the next generation of individuals.

            @current_population: (((int, int) list) list) Current population.

            @return: (((int, int) list) list) Next generation population.
        '''

        # Elitism
        new_population = []
        for i in range(int(self.ELIT_PCT * self.POP_SIZE)):
            fittest = self.get_fittest_chromosome(current_population)
            new_population.append(fittest)

        # Genetic operators
        while len(new_population) < self.POP_SIZE:
            parent1, parent2 = self.get_random_parents(current_population)

            # Crossover
            child1, child2 = self.crossover(parent1, parent2) \
                if np.random.uniform() < self.CX_PROB else \
                [tuple(g) for g in parent1], [tuple(g) for g in parent2]

            # Mutation
            child1 = self.mutate(
                child1) if np.random.uniform < self.MUT_PROB else child1

            child2 = self.mutate(
                child2) if np.random.uniform < self.MUT_PROB else child2

            new_population += [child1, child2]

        return new_population

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

    def mutate(self, chromosome):
        '''
            Mutate an individual's genes.

            @chromosome: ((int, int) list) Chromosome to mutate.

            @return: ((int, int) list) Mutated chromosome.
        '''

        # Get a random vertex (excluding start and end points)
        mutate_point = np.random.choice(range(len(chromosome[1:-1])))
        mutate_vertex = chromosome[1:-1][mutate_point]
        mutate_point = chromosome.index(mutate_vertex)

        neighbour_indices = get_neighbours(self.sampling_points, mutate_vertex)

        # Create mutations
        mutation = [tuple(t) for t in chromosome]
        mutations = [[tuple(t) for t in chromosome]]
        for vertex in neighbour_indices:
            # Ignore neighbour vertices already in chromosome
            if index in chromosome:
                continue

            mutation[mutate_point] = index

            # Ignore mutations that violate the cost limit
            if self.get_path_cost(mutation) > self.max_cost:
                continue

            mutations.append([tuple(t) for t in mutation])

        return self.get_fittest_chromosome(mutations)

    def evaluate_chromosome(self, chromosome):
        '''
            Calculate how fit an individual is, based on their genes.

            @chromosome: ((int, int) list) Chromosome to calculate fitness for.

            @return: (float) Chromosome fitness.
        '''

        reward = sum([self.sampling_points(gene) for gene in chromosome])

        return (reward ** 3) / self.get_path_cost(chromosome)

    def get_fittest_chromosome(self, population):
        '''
            Get the fittest chromosome in a population.

            @population: (((int, int) list) list) Population of individuals.

            @return: ((int, int) list) Fittest chromosome in the population.
        '''

        by_fitness = sorted(
            population, key=self.evaluate_chromosome, reverse=True)

        return by_fitness[0]
