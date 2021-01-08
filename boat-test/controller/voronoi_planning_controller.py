import heapq
from collections import defaultdict

import numpy as np
import pygame

from scipy.spatial import Voronoi

from controller.base_controller import BaseController
from boat_simulation.simple import Action, latlon_to_xy, BOAT_HEIGHT
from boat_simulation.latlon import LatLon

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

    def dist(self, a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def compute_voronoi(self, env):
        obstacle_coords = [obs.curr_coords for obs in env.obstacles]
        boat_xy = list(latlon_to_xy(env.boat_coords))
        waypoint_xy = list(latlon_to_xy(env.waypoints[self.curr_waypoint]))
        obstacle_coords.append(boat_xy)  # boat is always at second-to-last index
        obstacle_coords.append(waypoint_xy)  # curr waypoint is always at last index

        if len(obstacle_coords) < 4:
            return VoronoiGraph([], [])

        vor = Voronoi(obstacle_coords)

        center = vor.points.mean(axis=0)

        points = vor.vertices.tolist()

        # edges = [[] for i in range(len(points)+10)]
        edges = defaultdict(dict)

        for i in range(len(vor.ridge_vertices)):
            edge = vor.ridge_vertices[i]

            if edge[0] >= 0 and edge[1] >= 0:
                start = edge[0]
                end = edge[1]
            else:
                # we need to approximate the unknown vertex
                if edge[1] < 0:
                    # always make edge[0] the unknown vertex
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
                endpt = endpt.tolist()
                points.append(endpt)

                start = edge[1]
                end = len(points) - 1

            d = self.dist(points[start], points[end])
            edges[start][end] = d
            edges[end][start] = d

        points.append(boat_xy)
        points.append(waypoint_xy)

        # add edges from boat to vertices in its region
        for i in vor.regions[vor.point_region[len(obstacle_coords) - 2]]:
            if i >= 0:
                d = self.dist(points[len(points) - 2], points[i])
                edges[len(points) - 2][i] = d
                edges[i][len(points) - 2] = d

        # add edges from waypoint to vertices in its region
        for i in vor.regions[vor.point_region[len(obstacle_coords) - 1]]:
            if i >= 0:
                d = self.dist(points[len(points) - 1], points[i])
                edges[len(points) - 1][i] = d
                edges[i][len(points) - 1] = d

        return VoronoiGraph(points, edges)

    def compute_shortest_path(self, graph):
        if len(graph.points) < 2:
            return 1e9, None

        # start index is at boat, second-to-last point
        start = len(graph.points) - 2

        # end index is at waypoint, last point
        end = len(graph.points) - 1

        visited = [False] * len(graph.points)
        parent = [-1] * len(graph.points)
        dist = [1e9] * len(graph.points)

        dist[start] = 0
        parent[start] = start

        q = [(0, start)]
        heapq.heapify(q)

        while len(q) > 0:
            curr_dist, curr_point = heapq.heappop(q)
            if not visited[curr_point]:
                visited[curr_point] = True
                if curr_point == end:
                    break

                for neighbor in graph.edges[curr_point]:
                    if visited[neighbor]:
                        continue
                    new_dist = curr_dist + graph.edges[curr_point][neighbor]
                    if new_dist < dist[neighbor]:
                        dist[neighbor] = new_dist
                        parent[neighbor] = curr_point
                        heapq.heappush(q, (new_dist, neighbor))

        path = [end]
        curr = end
        while curr != start:
            curr = parent[curr]
            path.append(curr)

        path.reverse()

        return dist[end], path

    # uses ground truth state
    def select_action_from_state(self, env, state):
        if self.in_sim:
            env.set_waypoint(self.curr_waypoint)

        env.voronoi_graph = self.compute_voronoi(env)
        _, path = self.compute_shortest_path(env.voronoi_graph)
        env.voronoi_path = path

        return Action(0, 0)


class VoronoiGraph(object):
    def __init__(self, points, edges):
        self.points = points
        self.edges = edges  # represented as adjacency list
