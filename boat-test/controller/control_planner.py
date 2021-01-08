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

class ControlState(object):
    def __init__(self, accel, alpha, x, y, v, theta, omega, prev=None):
        self.accel = accel
        self.alpha = alpha
        self.x = x
        self.y = y
        self.prev = prev
        self.v = v
        self.omega = omega
        self.theta = theta

    def __lt__(self, other):
        return self.accel**2 + self.alpha**2 < other.accel**2 + other.alpha**2


# Boat is modelled as a rod with two thrusters on each end
class ControlPlanner(BaseController):
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

        self.start_x = None
        self.start_y = None

        self.path = None
        self.cmd_idx = 0
        self.cmd_start_t = 0

        self.delta_t = 1


    def get_distances(self, waypoint, boat_x, boat_y):
        """
        Given a LatLon waypoint, a lon boat_x, and a lat boat_y, outputs
        the delta x and delta y needed to get to the waypoint (signed and in meters)
        """
        x_targ, y_targ = waypoint[0], waypoint[1]
        x_curr, y_curr = boat_x, boat_y

        delta_x = LatLon.dist(LatLon(y_targ, x_curr), LatLon(y_targ, x_targ))
        if x_targ < x_curr:
            delta_x *= -1

        delta_y = LatLon.dist(LatLon(y_curr, x_targ), LatLon(y_targ, x_targ))
        if y_targ < y_curr:
            delta_y *= -1

        return delta_x, delta_y


    def check_intersecting(self, x, y, state):
        """
        Checks if a point x and y away from LatLon(self.start_y, self.start_x)
        intersects with any objects
        """
        boat_x, boat_y, obstacles = state[0], state[1], state[-1]
        test_pos = LatLon(self.start_y, self.start_x).add_dist(x, y)

        for obs in obstacles:
            obs_pos = (obs[1], obs[2])
            # delta_x, delta_y = self.get_distances(obs_pos, test_pos.lon, test_pos.lat)
            # print(delta_x, delta_y)
            if LatLon.dist(LatLon(obs_pos[1], obs_pos[0]), test_pos) < obs[0] + 2*BOAT_HEIGHT:
                # print("returned true")
                return True

        return False


    def theta(self, time, boat_angle, boat_ang_vel, alpha_applied):
        return boat_angle + boat_ang_vel * time + .5 * alpha_applied * (time**2)

    def change_in_x(self, time, boat_speed, accel_applied, boat_angle, boat_ang_vel, alpha_applied):
        return -(boat_speed + accel_applied * time) * np.sin(np.deg2rad(self.theta(time, boat_angle, boat_ang_vel, alpha_applied)))

    def change_in_y(self, time, boat_speed, accel_applied, boat_angle, boat_ang_vel, alpha_applied):
        return -(boat_speed + accel_applied * time) * np.cos(np.deg2rad(self.theta(time, boat_angle, boat_ang_vel, alpha_applied)))


    def deltas_travelled(self, accel_applied, alpha_applied, boat_angle, boat_ang_vel, boat_speed, dt=None):
        if dt is None:
            dt = self.delta_t
        x_fn = lambda time: self.change_in_x(time, boat_speed, accel_applied, boat_angle, boat_ang_vel, alpha_applied)
        y_fn = lambda time: self.change_in_y(time, boat_speed, accel_applied, boat_angle, boat_ang_vel, alpha_applied)

        delta_x_tot = integrate.quad(x_fn, 0, dt)[0]
        delta_y_tot = integrate.quad(y_fn, 0, dt)[0]

        return delta_x_tot, delta_y_tot


    def next_accel_states(self, delta_x, delta_y, obs, curr_state=None, prev_cost=0, cx=0, cy=0, ox=0, oy=0):
        num_accel = 10
        num_alpha = 10

        delta_t = 0.25     # seconds

        accel_grid_size = 2 * self.a_max / (num_accel - 1)
        alpha_grid_size = 2 * self.max_alpha_mag / (num_accel - 1)

        possible_next = []

        curr_vel = 0
        curr_ang_vel = 0
        curr_ang = 0

        if curr_state is not None:
            curr_vel = curr_state.v
            curr_ang_vel = curr_state.omega
            curr_ang = curr_state.theta

        for i in range(num_accel):
            accel_applied = -self.a_max + (i * accel_grid_size)
            if not (accel_applied == 0 and abs(curr_vel) < 3):
                for j in range(num_alpha):
                    alpha_applied = -self.max_alpha_mag + (j * alpha_grid_size)

                    delta_x_tot, delta_y_tot = self.deltas_travelled(accel_applied, alpha_applied, curr_ang, curr_ang_vel, curr_vel)
                    new_x = delta_x_tot + cx
                    new_y = delta_y_tot + cy

                    new_state = ControlState(accel_applied, alpha_applied, x=new_x, y=new_y, v=curr_vel + self.delta_t*accel_applied,
                        theta=curr_ang + curr_ang_vel*self.delta_t + 0.5*alpha_applied*(self.delta_t**2), omega=curr_ang_vel + self.delta_t*alpha_applied, prev=curr_state)

                    f = np.sqrt(delta_x_tot**2 + delta_y_tot**2) + prev_cost
                    g = np.sqrt((delta_x - new_x)**2 + (delta_y - new_y)**2) #+ abs(new_state.v)

                    if not self.check_intersecting(new_x, new_y, obs):
                        possible_next.append((f + g, new_state, f))

        return possible_next


    def accel_a_star(self, start_x, start_y, delta_x, delta_y, obs):
        frontier = self.next_accel_states(delta_x, delta_y, obs, cx=start_x, cy=start_y)
        visited = set((start_x, start_y))

        heapq.heapify(frontier)     # Make frontier a priority queue
        # print(frontier)

        while len(frontier) > 0:
            val_new, s_new, f_new = heapq.heappop(frontier)
            control_new = (s_new.accel, s_new.alpha)

            pos_new = (s_new.x, s_new.y)

            # We reached the goal location
            dist_to = np.sqrt((delta_x - pos_new[0])**2 + (delta_y - pos_new[1])**2)
            print(f"pos_new: {pos_new}, dist, v, omega: {(dist_to, s_new.v, s_new.omega)}")
            if np.sqrt((delta_x - pos_new[0])**2 + (delta_y - pos_new[1])**2) < 1:
                print(f"deltas: {(delta_x, delta_y)}, curr: {pos_new}")
                return s_new

            # We haven't reached the goal, so compute all next states we can go to
            possible = self.next_accel_states(delta_x, delta_y, obs, curr_state=s_new, prev_cost=f_new, cx=pos_new[0], cy=pos_new[1])

            # If we haven't seen the state before, add it to the queue
            for k in possible:
                heapq.heappush(frontier, k)


    def draw(self, delta_x, delta_y, path, state, path_without=None, controls=False):
        """
        Plots path and obstacles on matplotlib plot.
        """
        fig, ax = plt.subplots()
        grid_res = 1

        ax.plot([0], [0], c='b', marker='o')
        ax.plot([delta_x], [-delta_y], c='g', marker='o')

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

        curr_pos = (0, 0)
        curr_ang = 0
        curr_vel = 0
        curr_ang_vel = 0
        # path is a list of ControlStates in order applied
        for c in path:
            a, alpha = c.accel, c.alpha
            xs = []
            ys = []

            for t in np.linspace(0, self.delta_t, 60):
                dx, dy = self.deltas_travelled(a, alpha, curr_ang, curr_ang_vel, curr_vel, t)
                xs.append(curr_pos[0] + dx)
                ys.append(-curr_pos[1] - dy)

            plt.plot(xs, ys)

            curr_pos = (c.x, c.y)
            curr_ang = c.theta
            curr_vel = c.v
            curr_ang_vel = c.omega
        #
        # if path_without is not None:
        #     xs = [0] + [w[0] for w in path_without]
        #     ys = [0] + [w[1] for w in path_without]
        #
        #     plt.plot(xs, ys)
        #
        #
        # obstacles = state[-1]
        #
        # for obs in obstacles:
        #     obs_pos = (obs[1], obs[2])
        #     ox, oy = self.get_distances(obs_pos, state[0], state[1])
        #     obs_circle = plt.Circle((ox, oy), obs[0], color='g')
        #     ax.add_artist(obs_circle)
        #
        # ax.set_yticks(ty, minor=False)
        # ax.set_xticks(tx, minor=False)
        #
        # ax.yaxis.grid(True, which='major')
        # ax.xaxis.grid(True, which='major')

        plt.show()


    # uses ground truth state
    def select_action_from_state(self, env, state):
        """
        Main method that performs A* search, selects subgoal to go to,
        and outputs which actions need to be taken.
        """

        if self.in_sim:
            env.set_waypoint(self.curr_waypoint)

        if env.total_time < 1:
            self.path = None
            self.start_x = None
            self.start_y = None
            self.subgoal_idx = 0
            return Action(0, 0)

        boat_x, boat_y, boat_speed, _, boat_angle, boat_ang_vel, ocean_current_x, ocean_current_y, obstacles = state

        waypoint = [env.waypoints[self.curr_waypoint].lon, env.waypoints[self.curr_waypoint].lat]

        if self.start_x is None:
            self.start_x = boat_x
            self.start_y = boat_y

        delta_x, delta_y = self.get_distances(waypoint, self.start_x, self.start_y)
        boat_x, boat_y = self.get_distances([boat_x, boat_y], self.start_x, self.start_y)

        dist = np.sqrt((delta_x - boat_x)**2 + (delta_y - boat_y)**2)

        if dist < 0.05:
            self.curr_waypoint = (self.curr_waypoint + 1) % len(env.waypoints)
            self.path = None
            self.start_x = None
            self.start_y = None
            self.cmd_idx = 0
            return Action(0, 0)

        if self.path is None:
            curr = self.accel_a_star(boat_x, boat_y, delta_x, delta_y, state)
            path_rev = []
            path_cstates_rev = []

            while curr is not None:
                path_rev.append((curr.accel, curr.alpha))
                path_cstates_rev.append(curr)
                curr = curr.prev

            self.path = path_rev

            print(self.path)
            path_cstates_rev.reverse()
            self.draw(delta_x, delta_y, path_cstates_rev, state, controls=True)

            self.cmd_idx = 0
            self.cmd_start_t = env.total_time

        if env.total_time - self.cmd_start_t >= self.delta_t:
            self.cmd_idx += 1
            print("switching")
            self.cmd_start_t = env.total_time

        if self.cmd_idx < len(self.path):
            control = self.path[len(self.path) - 1 - self.cmd_idx]
            return Action(2, [control[1], control[0]])

        return Action(0, 0)
