import numpy as np
import pygame

from scipy.spatial import Voronoi

from controller.base_controller import BaseController
from boat_simulation.simple import Action, latlon_to_xy, BOAT_HEIGHT
from boat_simulation.latlon import LatLon

import matplotlib.pyplot as plt
import heapq

VEL_SCALE = 1 / 60


# Boat is modelled as a rod with two thrusters on each end
class VoronoiPlanningController(BaseController):
    def __init__(self, in_sim=True, print_info=True):
        BaseController.__init__(self, "voronoi_planning")
        self.in_sim = in_sim

        self.f_max = 50
        self.boat_mass = 5
        self.boat_width = 0.5

        self.a_max = 2 * self.f_max / (self.boat_mass)
        self.max_alpha_mag = 6 * self.f_max / (self.boat_mass * self.boat_width)

        self.curr_waypoint = 0

        self.print_info = print_info

        self.p_scale = np.array([1.5, 0, 1.5]).reshape(3, 1)

        self.grid_size = 1

        self.start_x = None
        self.start_y = None
        self.path = []

        self.replot = False
        self.subgoal_idx = 0

    def compute_voronoi(self, env):
        obstacle_coords = [obs.curr_coords for obs in env.obstacles]
        if len(obstacle_coords) < 4:
            return VoronoiGraph([], [])
        # print(obstacle_coords)

        vor = Voronoi(obstacle_coords)

        center = vor.points.mean(axis=0)

        points = vor.vertices.tolist()
        edges = []

        for i in range(len(vor.ridge_vertices)):
            edge = vor.ridge_vertices[i]

            if edge[0] >= 0 and edge[1] >= 0:
                start = edge[0]
                end = edge[1]
                # edges.append([start, end])
            else:
                if edge[1] < 0:
                    edge[0], edge[1] = edge[1], edge[0]

                between_pts = vor.ridge_points[i]
                p1 = between_pts[0]
                p2 = between_pts[1]

                # taken from lines 66-72 of https://gist.github.com/pv/8036995

                t = vor.points[p2] - vor.points[p1]  # tangent
                t /= np.linalg.norm(t)
                n = np.array([-t[1], t[0]])  # normal

                midpoint = vor.points[[p1, p2]].mean(axis=0)
                direction = np.sign(np.dot(midpoint - center, n)) * n

                endpt = vor.vertices[edge[1]] + direction * 1000
                points.append(endpt)

                start = edge[1]
                end = len(points) - 1

            edges.append([start, end])

        print(points)
        print(edges)
        return VoronoiGraph(points, edges)

    # uses ground truth state
    def select_action_from_state(self, env, state):
        """
        Main method that performs A* search, selects subgoal to go to,
        and outputs which actions need to be taken.
        """
        env.voronoi_graph = self.compute_voronoi(env)
        return Action(0, 0)


class VoronoiGraph(object):
    def __init__(self, points, edges):
        self.points = points
        self.edges = edges
