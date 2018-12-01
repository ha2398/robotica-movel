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
from time import time


class GeneticAlgorithm:

    TOURNAMENT_SIZE = 7  # Number of individuals in tournament selection
    CX_PROB = 0.8  # Crossover probability
    MUT_PROB = 0.5  # Mutation probability
    ELIT_PCT = 0.05  # Elitist percentage
    IMPR_THRESHOLD = 0.1  # Improvement threshold

    def __init__(self, sampling_points, max_cost, generations, pop_size):
        '''
            Initialize the OP GA instance.

            @sampling_points: (2D numpy array) Array that represents the
                sampling points in the environment and their rewards.
            @max_cost: (float) Max path cost.
            @generations: (int) Number of generations to run the GA for.
            @pop_size: (int) Number of individuals in each population.
        '''

        self.sampling_points = sampling_points
        self.max_cost = max_cost
        self.generations = generations
        self.pop_size = pop_size

    def run(self, start, finish):
        '''
            Run the genetic algorithm instance.

            @start: (float, float) Robot's path start point (coordinates).
            @finish: (float, float) Robot's path end point (coordinates).

            @return:
                ((int, int) list) Fittest chromosome.
                (float) Fitness of fittest chromosome.
        '''

        self.fitness_history = {}  # Prevents repeated calculations

        start = position_to_index(start[0], start[1], self.sampling_points)
        finish = position_to_index(finish[0], finish[1], self.sampling_points)

        file = open('stats.csv', 'w')
        file.write('Generation,Best fitness,Fittest cost, Fittest reward,Time elapsed\n')

        population = self.initialize_population(start, finish)
        generation = 0
        gens_without_improvement = 0
        last_fitness = None

        while generation < self.generations and gens_without_improvement < 10:
            print 'Generation', generation
            t0 = time()

            population = self.select_new_population(population)
            fittest = self.get_fittest_chromosome(population)
            current_fitness = self.evaluate_chromosome(fittest)

            if last_fitness is not None:
                if abs(current_fitness - last_fitness) <= self.IMPR_THRESHOLD:
                    gens_without_improvement += 1
                else:
                    gens_without_improvement = 0

            last_fitness = current_fitness
            c = self.get_path_cost(fittest)
            r = self.get_total_reward(fittest)

            time_elapsed = time() - t0

            print 'Best fitness:', round(current_fitness, 2)
            print 'Fittest cost:', round(c, 2)
            print 'Fittest reward:', round(r, 2)
            print 'Time elapsed:', round(time_elapsed, 2), 's'

            file.write('{},{},{},{},{}\n'.format(generation,
                                                 round(current_fitness, 2),
                                                 round(c, 2),
                                                 round(r, 2),
                                                 round(time_elapsed, 2)))

            generation += 1
            print

        file.close()

        return [index_to_center_of_mass(v[0], v[1], self.sampling_points)
                for v in fittest], self.get_path_cost(fittest), self.get_total_reward(fittest)

    def get_path_cost(self, path):
        '''
            Get the total cost of a path.

            @path: ((int, int) list) Path to get cost for.

            @return: (float) Path cost.
        '''

        cost = 0
        for start, end in list(zip(path, path[1:])):
            p1 = np.array(index_to_center_of_mass(start[0], start[1],
                                                  self.sampling_points))
            p2 = np.array(index_to_center_of_mass(end[0], end[1],
                                                  self.sampling_points))

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
        map_dim = self.sampling_points.shape

        while len(population) < self.pop_size:
            chromosome = [start]

            # Retrieve available vertices.
            vertices = []
            for i in range(map_dim[0]):
                for j in range(map_dim[1]):
                    if self.sampling_points[(i, j)] != 1:
                        vertices.append((i, j))

            done = False
            while not done and len(vertices) != 0:
                v = np.random.choice(range(len(vertices)))
                v = vertices[v]
                insertion_cost = self.get_path_cost(chromosome + [v, finish])

                if insertion_cost <= self.max_cost:
                    chromosome.append(v)
                    vertices.remove(v)
                else:
                    done = True

            chromosome.append(finish)
            population.append(chromosome)

        return population

    def tournament_selection(self, population):
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

    def get_random_parents(self, population):
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

        return parent1, parent2

    def select_new_population(self, current_population):
        '''
            Select the new population for the next generation of individuals.

            @current_population: (((int, int) list) list) Current population.

            @return: (((int, int) list) list) Next generation population.
        '''

        # Elitism
        new_population = []
        elit_size = int(self.ELIT_PCT * self.pop_size)
        for individual in self.get_fittest_chromosome(current_population,
                                                      elit_size):
            new_population.append(individual)

        # Genetic operators
        while len(new_population) < self.pop_size:
            parent1, parent2 = self.get_random_parents(current_population)

            # Crossover
            if np.random.uniform < self.CX_PROB:
                child1, child2 = self.crossover(parent1, parent2)
            else:
                child1 = [tuple(g) for g in parent1]
                child2 = [tuple(g) for g in parent2]

            # Mutation
            if np.random.uniform() < self.MUT_PROB:
                child1 = self.mutate(child1)
                child2 = self.mutate(child2)

            for child in [child1, child2]:
                if self.get_path_cost(child) <= self.max_cost:
                    new_population.append(child)

        return new_population

    def remove_duplicated_genes(self, chromosome):
        '''
            Remove duplicated genes in a chromosome.

            @chromosome: ((int, int) list) Chromosome to remove duplicates on.

            @return: ((int, int) list) The chromosome passed as argument, but
                without any duplicated genes.
        '''

        new_chromosome = []
        for gene in chromosome:
            if gene not in new_chromosome:
                new_chromosome.append(gene)

        return new_chromosome

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

            # Remove duplicates
            child1 = self.remove_duplicated_genes(child1)
            child2 = self.remove_duplicated_genes(child2)

            return child1, child2
        else:  # No, they don't.
            return parent1, parent2

    def mutate(self, chromosome):
        '''
            Mutate an individual's genes.

            @chromosome: ((int, int) list) Chromosome to mutate.

            @return: ((int, int) list) Mutated chromosome.
        '''

        if len(chromosome) <= 3:
            return chromosome

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
            if vertex in chromosome:
                continue

            mutation[mutate_point] = vertex

            # Ignore mutations that violate the cost limit
            if self.get_path_cost(mutation) > self.max_cost:
                continue

            mutations.append([tuple(t) for t in mutation])

        return self.get_fittest_chromosome(mutations)

    def get_total_reward(self, chromosome):
        '''
            Get the total reward the robot would get by visiting all vertices
            in the chromosome.

            @chromosome: ((int, int) list) Chromosome to calculate rewards for.

            @return: (float) Sum of rewards.
        '''

        return sum([self.sampling_points[gene] for gene in chromosome])

    def evaluate_chromosome(self, chromosome):
        '''
            Calculate how fit an individual is, based on their genes.

            @chromosome: ((int, int) list) Chromosome to calculate fitness for.

            @return: (float) Chromosome fitness.
        '''

        chromosome_str = str(chromosome)

        if chromosome_str not in self.fitness_history:
            reward = self.get_total_reward(chromosome)
            fitness = (reward ** 3) / self.get_path_cost(chromosome)
            self.fitness_history[chromosome_str] = fitness
            # self.fitness_history[chromosome_str] = reward
        else:
            fitness = self.fitness_history[chromosome_str]

        return fitness

    def get_fittest_chromosome(self, population, number=1):
        '''
            Get the fittest chromosome in a population.

            @population: (((int, int) list) list) Population of individuals.
            @number: (int) Number of fittest individuals to select.

            @return: ((int, int) list) Fittest chromosome in the population.
        '''

        by_fitness = sorted(
            population, key=self.evaluate_chromosome, reverse=True)

        if number == 1:
            return by_fitness[0]
        else:
            return by_fitness[:number]
