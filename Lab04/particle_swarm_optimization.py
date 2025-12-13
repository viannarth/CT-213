import numpy as np
import random
from math import inf


class Particle:
    """
    Represents a particle of the Particle Swarm Optimization algorithm.
    """
    def __init__(self, lower_bound, upper_bound):
        """
        Creates a particle of the Particle Swarm Optimization algorithm.

        :param lower_bound: lower bound of the particle position.
        :type lower_bound: numpy array.
        :param upper_bound: upper bound of the particle position.
        :type upper_bound: numpy array.
        """
        self.position = np.random.uniform(lower_bound, upper_bound)
        delta = upper_bound - lower_bound
        self.velocity = np.random.uniform(-delta, delta)
        self.value = None
        self.best_position = None
        self.best_value = -inf
        self.was_evaluated = False # Determines if the particle was evaluated in the current generation


class ParticleSwarmOptimization:
    """
    Represents the Particle Swarm Optimization algorithm.
    Hyperparameters:
        inertia_weight: inertia weight.
        cognitive_parameter: cognitive parameter.
        social_parameter: social parameter.

    :param hyperparams: hyperparameters used by Particle Swarm Optimization.
    :type hyperparams: Params.
    :param lower_bound: lower bound of particle position.
    :type lower_bound: numpy array.
    :param upper_bound: upper bound of particle position.
    :type upper_bound: numpy array.
    """
    def __init__(self, hyperparams, lower_bound, upper_bound):
        self.particles = []
        for _ in range(hyperparams.num_particles):
            self.particles.append(Particle(lower_bound, upper_bound))
        self.w = hyperparams.inertia_weight
        self.phip = hyperparams.cognitive_parameter
        self.phig = hyperparams.social_parameter
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound

    def get_best_position(self):
        """
        Obtains the best position so far found by the algorithm.

        :return: the best position.
        :rtype: numpy array.
        """
        best_position = None
        best_value = -inf
        for particle in self.particles:
            if particle.best_value > best_value:
                best_value = particle.best_value
                best_position = particle.position
        return best_position

    def get_best_value(self):
        """
        Obtains the value of the best position so far found by the algorithm.

        :return: value of the best position.
        :rtype: float.
        """
        best_value = -inf
        for particle in self.particles:
            if particle.best_value > best_value:
                best_value = particle.best_value
        return best_value

    def get_position_to_evaluate(self):
        """
        Obtains a new position to evaluate.

        :return: position to evaluate.
        :rtype: numpy array.
        """
        for i in range(len(self.particles)):
            particle = self.particles[i]
            if not particle.was_evaluated:
                return particle.position

    def advance_generation(self):
        """
        Advances the generation of particles. Auxiliary method to be used by notify_evaluation().
        """
        # Ensuring that all the particles were evaluated in the current generation
        for particle in self.particles:
            if not particle.was_evaluated:
                return
        for particle in self.particles:
            rp = random.uniform(0.0, 1.0)
            rg = random.uniform(0.0, 1.0)
            particle.velocity = self.w * particle.velocity + self.phip * rp * (particle.best_position - particle.position) + self.phig * rg * (self.get_best_position() - particle.position)
            particle.position = particle.position + particle.velocity
            # Limit the position and the velocity within the boundaries
            particle.position = np.minimum(np.maximum(particle.position, self.lower_bound), self.upper_bound)
            delta = self.upper_bound - self.lower_bound
            velocity_ub = delta
            velocity_lb = -delta
            particle.velocity = np.minimum(np.maximum(particle.velocity, velocity_lb), velocity_ub)
            # Set all particles as not evaluated for the next generation
            particle.was_evaluated = False

    def notify_evaluation(self, value):
        """
        Notifies the algorithm that a particle position evaluation was completed.

        :param value: quality of the particle position.
        :type value: float.
        """
        for i in range(len(self.particles)):
            particle = self.particles[i]
            if not particle.was_evaluated:
                particle.value = value
                particle.was_evaluated = True
                if value > particle.best_value:
                    particle.best_value = value
                    particle.best_position = particle.position
                self.advance_generation()
                return