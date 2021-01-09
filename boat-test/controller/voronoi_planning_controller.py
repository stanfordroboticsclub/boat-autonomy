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

        self.path = []
        self.voronoi_graph = None


    def dist(self, a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def compute_voronoi(self, obstacles, boat_latlon, waypoint_latlon):
        obstacles_xy = []
        for obs in obstacles:
            obs_latlon = LatLon(obs[2], obs[1])
            obstacles_xy.append(list(latlon_to_xy(obs_latlon)))

        boat_xy = list(latlon_to_xy(boat_latlon))
        waypoint_xy = list(latlon_to_xy(waypoint_latlon))

        voronoi_points = obstacles_xy  # obstacle LatLon objects
        voronoi_points.append(boat_xy)  # boat is always at second-to-last index
        voronoi_points.append(waypoint_xy)  # curr waypoint is always at last index

        if len(voronoi_points) < 4:
            return VoronoiGraph([], [])

        vor = Voronoi(voronoi_points)

        center = vor.points.mean(axis=0)

        points = vor.vertices.tolist()

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

        # if the boat and waypoint regions border each other, add a connecting edge
        for edge in vor.ridge_points:
            p1 = edge[0]
            p2 = edge[1]
            if min(p1, p2) == len(vor.points) - 2 and max(p1, p2) == len(vor.points) - 1:
                d = self.dist(vor.points[p1], vor.points[p2])
                edges[len(points) - 2][len(points) - 1] = d
                edges[len(points) - 1][len(points) - 2] = d

        # add edges from boat to vertices in its region
        for i in vor.regions[vor.point_region[len(voronoi_points) - 2]]:
            if i >= 0:
                d = self.dist(points[len(points) - 2], points[i])
                edges[len(points) - 2][i] = d
                edges[i][len(points) - 2] = d

        # add edges from waypoint to vertices in its region
        for i in vor.regions[vor.point_region[len(voronoi_points) - 1]]:
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

    def get_required_angle_change(self, boat_angle, delta_x, delta_y):
        """
        Get the change in angle required to turn to a point delta x and delta y
        away (delta x and delta y are in meters)
        """
        angle = (np.arctan2(-delta_x, -delta_y) * 180 / np.pi) - (boat_angle)
        angle = angle % 180
        angle = min(angle, angle - 180, key=abs)
        return angle

    def control(self, boat_angle, delta_x, delta_y, boat_speed, boat_ang_vel):
        """
        Get controls needed to steer boat to a particular point.
        This function is used to follow the path planned using A*
        """

        boat_angle_deg = np.deg2rad(boat_angle)
        R = np.array([  [-np.sin(boat_angle_deg),   np.cos(boat_angle_deg) ,    0],
                        [-np.cos(boat_angle_deg),  -np.sin(boat_angle_deg),     0],
                        [0,                         0,                          1]]).reshape((3, 3))
        R_inv = R.T

        angle = self.get_required_angle_change(boat_angle, delta_x, delta_y)

        eta_d = np.array([delta_x, delta_y, angle]).reshape((3, 1))

        targ_v = self.p_scale * np.matmul(R_inv, eta_d)
        curr_v = np.array([boat_speed, 0, boat_ang_vel]).reshape((3, 1))
        diff_v = targ_v - curr_v

        control = self.p_scale * diff_v
        control[2][0] = np.rad2deg(control[2][0])
        control = np.clip(control, np.array([-self.a_max, 0, -self.max_alpha_mag]).reshape(3, 1), np.array([self.a_max, 0, self.max_alpha_mag]).reshape(3, 1))

        dist = np.sqrt((delta_x ** 2) + (delta_y ** 2))

        # print(f"self.running_dist_err: {self.running_dist_err}, self.running_angle_err: {self.running_angle_err}")
        if self.print_info:
            print(f"dist: {round(dist, 5)},  curr_vel: {round(boat_speed, 5)}, accel: {round(control[0][0], 5)}, alpha: {round(control[2][0], 5)}")

        return Action(2, [control[2][0], control[0][0]])

    # uses ground truth state
    def select_action_from_state(self, env, state):
        """
        Main method that calculates Voronoi diagram, finds shortest path,
        and outputs which actions need to be taken.
        """
        if self.in_sim:
            env.set_waypoint(self.curr_waypoint)

        if env.total_time < 1:
            self.path = []
            env.voronoi_graph = None
            env.voronoi_path = None
            return Action(0, 0)

        boat_x, boat_y, boat_speed, _, boat_angle, boat_ang_vel, ocean_current_x, ocean_current_y, obstacles = state

        boat_latlon = LatLon(boat_y, boat_x)
        waypoint_laton = env.waypoints[self.curr_waypoint]

        dist = LatLon.dist(boat_latlon, waypoint_laton)

        if dist < 0.05:
            self.curr_waypoint = (self.curr_waypoint + 1) % len(env.waypoints)
            self.path = None
            return Action(0, 0)

        self.voronoi_graph = self.compute_voronoi(obstacles, boat_latlon, waypoint_laton)
        _, self.path = self.compute_shortest_path(self.voronoi_graph)

        env.voronoi_graph = self.voronoi_graph
        env.voronoi_path = self.path

        if self.path is None:
            return Action(0, 0)

        target_point = self.voronoi_graph.points[self.path[1]]
        boat_xy = list(latlon_to_xy(boat_latlon))

        return self.control(boat_angle, target_point[0] - boat_xy[0], target_point[1] - boat_xy[1], boat_speed, boat_ang_vel)
        # return Action(0,0)


class VoronoiGraph(object):
    def __init__(self, points, edges):
        self.points = points
        self.edges = edges  # represented as adjacency list
