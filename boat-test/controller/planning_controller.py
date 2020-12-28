import numpy as np
import pygame

from scipy.optimize import minimize, Bounds, differential_evolution
import scipy.integrate as integrate

from controller.base_controller import BaseController
from boat_simulation.simple import Action, latlon_to_xy, BOAT_HEIGHT
from boat_simulation.latlon import LatLon

import matplotlib.pyplot as plt
import heapq

VEL_SCALE = 1/60

# Boat is modelled as a rod with two thrusters on each end
class PlanningController(BaseController):
    def __init__(self, in_sim=True, print_info=True):
        BaseController.__init__(self, "planning")
        self.in_sim = in_sim

        self.f_max = 50
        self.boat_mass = 5
        self.boat_width = 0.5

        self.a_max = 2 * self.f_max / (self.boat_mass)
        self.max_alpha_mag = 6 * self.f_max / (self.boat_mass * self.boat_width)

        self.curr_waypoint = 0

        self.print_info = print_info

        self.p_scale = np.array([1.5, 0, 1.5]).reshape(3, 1)


    def get_distances(self, waypoint, boat_x, boat_y):
        x_targ, y_targ = waypoint[0], waypoint[1]
        x_curr, y_curr = boat_x, boat_y

        delta_x = LatLon.dist(LatLon(y_targ, x_curr), LatLon(y_targ, x_targ))
        if x_targ < x_curr:
            delta_x *= -1

        delta_y = LatLon.dist(LatLon(y_curr, x_targ), LatLon(y_targ, x_targ))
        if y_targ < y_curr:
            delta_y *= -1

        return delta_x, delta_y


    def get_required_angle_change(self, boat_angle, delta_x, delta_y):
        angle = (np.arctan2(-delta_x, -delta_y) * 180 / np.pi) - (boat_angle)
        angle = angle % 180
        angle = min(angle, angle - 180, key=abs)
        return angle


    def check_intersecting(self, x, y, state):
        boat_x, boat_y, obstacles = state[0], state[1], state[-1]
        test_pos = LatLon(boat_y, boat_x).add_dist(x, y)


        for obs in obstacles:
            obs_pos = (obs[1], obs[2])
            delta_x, delta_y = self.get_distances(obs_pos, test_pos.lon, test_pos.lat)
            # print(delta_x, delta_y)
            if np.sqrt(delta_x**2 + delta_y**2) < obs[0] + BOAT_HEIGHT:
                # print("returned true")
                return True

        return False


    def next_states(self, delta_x, delta_y, obs, curr_state=[], prev_cost=0, cx=0, cy=0, ox=0, oy=0):
        dist = np.sqrt((delta_x - cx)**2 + (delta_y - cy)**2)
        if dist <= 0.5*np.sqrt(2):
            g = np.sqrt((delta_x - ox)**2 + (delta_y - oy)**2)
            return [(dist + prev_cost, curr_state + [(delta_x, delta_y)], dist + prev_cost)]

        out = []
        options = [-0.5, 0, 0.5]
        for o1 in options:
            for o2 in options:
                if not (o1 == 0 and o2 == 0):
                    f = prev_cost + np.sqrt(o1**2 + o2**2)

                    x = cx + o1
                    y = cy + o2

                    g = np.sqrt((delta_x - x)**2 + (delta_y - y)**2)

                    tot_cost = f + g
                    if not self.check_intersecting(x, y, obs):
                        out.append((tot_cost, curr_state + [(x, y)], f))

        return out


    def frontier_contains(self, frontier, state):
        for k in frontier:
            if k[1][-1] == state[1][-1]:
                return True
        return False


    def a_star(self, delta_x, delta_y, obs):
        frontier = self.next_states(delta_x, delta_y, obs)
        heapq.heapify(frontier)
        visited = set((0, 0))

        while True:
            # print(f"frontier: {frontier}")
            val_new, s_new, f_new = heapq.heappop(frontier)
            # print(s_new)
            pos_new = s_new[-1]
            visited.add(pos_new)

            if pos_new[0] == delta_x and pos_new[1] == delta_y:
                return s_new

            possible = self.next_states(delta_x, delta_y, obs, s_new, prev_cost=f_new, cx=pos_new[0], cy=pos_new[1])

            if len(frontier) == 0 and len(possible) == 0:
                return s_new

            for k in possible:
                if k[1][-1] not in visited and not self.frontier_contains(frontier, k):
                    heapq.heappush(frontier, k)


    def draw(self, delta_x, delta_y, path, state, path_without=None):
        fig, ax = plt.subplots()
        grid_res = 0.5

        ax.plot([0], [0], c='b', marker='o')
        ax.plot([delta_x], [delta_y], c='g', marker='o')

        delta_y = int(delta_y)
        delta_x = int(delta_x)

        delta_y += np.sign(delta_y)
        delta_x += np.sign(delta_x)

        start_x = 0 - np.sign(delta_x)*grid_res
        start_y = 0 - np.sign(delta_y)*grid_res

        # ty = np.arange(min(start_y, delta_y), max(start_y, delta_y), grid_res)
        # tx = np.arange(min(start_x, delta_x), max(start_x, delta_x), grid_res)

        ty = np.arange(-10, 10, grid_res)
        tx = np.arange(-10, 10, grid_res)

        xs = [0] + [w[0] for w in path]
        ys = [0] + [w[1] for w in path]

        plt.plot(xs, ys)

        if path_without is not None:
            xs = [0] + [w[0] for w in path_without]
            ys = [0] + [w[1] for w in path_without]

            plt.plot(xs, ys)


        obstacles = state[-1]

        for obs in obstacles:
            obs_pos = (obs[1], obs[2])
            ox, oy = self.get_distances(obs_pos, state[0], state[1])
            obs_circle = plt.Circle((ox, oy), obs[0], color='g')
            ax.add_artist(obs_circle)

        ax.set_yticks(ty, minor=False)
        ax.set_xticks(tx, minor=False)

        ax.yaxis.grid(True, which='major')
        ax.xaxis.grid(True, which='major')

        plt.show()


    def control(self, boat_angle, delta_x, delta_y, boat_speed, boat_ang_vel):
        boat_angle_deg = np.deg2rad(boat_angle)
        R = np.array([  [-np.sin(boat_angle_deg),   np.cos(boat_angle_deg) ,    0],
                        [-np.cos(boat_angle_deg),  -np.sin(boat_angle_deg),    0],
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
        if self.in_sim:
            env.set_waypoint(self.curr_waypoint)

        if env.total_time < 1:
            return Action(0, 0)

        boat_x, boat_y, boat_speed, _, boat_angle, boat_ang_vel, ocean_current_x, ocean_current_y, obstacles = state
        waypoint = [env.waypoints[self.curr_waypoint].lon, env.waypoints[self.curr_waypoint].lat]

        delta_x, delta_y = self.get_distances(waypoint, boat_x, boat_y)
        dist = np.sqrt(delta_x**2 + delta_y**2)

        if dist < 0.05:
            self.curr_waypoint = (self.curr_waypoint + 1) % len(env.waypoints)
            self.running_dist_err = 0
            self.running_angle_err = 0

        # print(self.next_states(delta_x, delta_y))
        path_with = self.a_star(delta_x, delta_y, state)
        # no_obs = []
        # for k in state:
        #     no_obs.append(k)
        # no_obs[-1] = []
        # path_without = self.a_star(delta_x, delta_y, no_obs)

        # self.draw(delta_x, delta_y, path_with, state, path_without)

        return self.control(boat_angle, path_with[0][0], path_with[0][1], boat_speed, boat_ang_vel)
