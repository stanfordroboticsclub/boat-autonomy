import numpy as np
import pygame

from controller.base_controller import BaseController
from boat_simulation.simple import Action, latlon_to_xy, BOAT_HEIGHT
from boat_simulation.latlon import LatLon

import heapq


class PriorityQueue(object):
    """docstring for PriorityQueue."""

    def __init__(self):
        super(PriorityQueue, self).__init__()

    def insert(self, vertex, key):
        pass

    def contains(self, vertex):
        pass

    def remove(self, vertex):
        pass



class DStarPlanner(BaseController):
    def __init__(self, in_sim=True, print_info=True):
        BaseController.__init__(self, "d star planning", handle_quit=False)
        self.in_sim = in_sim

        self.f_max = 50
        self.boat_mass = 5
        self.boat_width = 0.5

        self.a_max = 2 * self.f_max / (self.boat_mass)
        self.max_alpha_mag = 6 * self.f_max / (self.boat_mass * self.boat_width)

        self.curr_waypoint = 0

        self.goal_state = None

        self.rhs_vals = {}
        self.g_vals = {}
        self.c_vals = {}

        self.intialized = False


    def rhs(self, state):
        return self.rhs_vals[state]


    def g(self, state):
        return self.g_vals[state]


    def h(self, state_1, state_2):
        return 0


    def c(self, s, s_prime):
        pass


    def predecessors(self, s):
        pass


    def successors(self, s):
        pass


    def calculate_key(self, state):
        return (min(self.g(state), self.rhs(state)) + self.h(state, self.goal_state),
            min(self.g(state), self.rhs(state)))


    def initialize(self, all_states):
        self.priority_queue = PriorityQueue()
        for state in all_states:
            self.g_vals[state] = self.rhs_vals[state] = np.inf
        self.rhs_vals[self.start_state] = 0
        self.priority_queue.insert(start_state, self.calculate_key(self.start_state))


    def compute_shortest_path(self):
        pass


    def update_vertex(self, u):
        if u != self.start_state:
            temp_key = lambda x: self.g(s) + self.c(s, u)
            self.rhs_vals[u] = temp_key(min(self.predecessors(u), key=temp_key))
        if self.priority_queue.contains(u):
            self.priority_queue.remove(u)
        if self.g(u) != self.rhs(u):
            self.priority_queue.insert(u, self.calculate_key(u))


    def get_changed_edge_costs(self):
        pass


    def main(self):
        if not self.initialized:
            self.initialize([])
            self.initialized = True

        self.compute_shortest_path()
        for edge_and_cost in self.get_changed_edge_costs():
            edge, new_edge_cost = edge_and_cost
            self.c_vals[edge] = new_edge_cost
            self.update_vertex(edge[1])
