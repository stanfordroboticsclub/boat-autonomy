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
        BaseController.__init__(self, "planning", handle_quit=False)
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

        # lng and lat of start pos
        self.start_lon = None
        self.start_lat = None

        # currently planned path
        self.path = []
        self.max_evals = 200

        # do we need to redraw the path
        self.replot = False
        self.subgoal_idx = 0

        # the last subgoal we were working towards
        self.last_subgoal_idx = 0

        # how far to keep away from obstacles
        self.clearance = 2*BOAT_HEIGHT


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


    def get_required_angle_change(self, boat_angle, delta_x, delta_y):
        """
        Get the change in angle required to turn to a point delta x and delta y
        away (delta x and delta y are in meters)
        """
        angle = (np.arctan2(-delta_x, -delta_y) * 180 / np.pi) - (boat_angle)
        angle = angle % 180
        angle = min(angle, angle - 180, key=abs)
        return angle


    def check_intersecting(self, x, y, state):
        """
        Checks if a point x and y away from LatLon(self.start_lat, self.start_lon)
        intersects with any objects
        """
        obstacles = state[-1]
        test_pos = LatLon(self.start_lat, self.start_lon).add_dist(x, y)

        for obs in obstacles:
            obs_pos = (obs[1], obs[2])
            if LatLon.dist(LatLon(obs_pos[1], obs_pos[0]), test_pos) < obs[0] + self.clearance:
                return True

        return False


    def next_states(self, delta_x, delta_y, observation, curr_state=[], prev_cost=0, cx=0, cy=0, ox=0, oy=0):
        """
        Given a goal to travel delta_x and delta_y, the current observation obs,
        'curr_state' which keeps track of all (x, y) visited so far, the cost of
        the previous position, the current position cx and cy, and the 'origin'
        ox and oy, output the possible positions to travel to next based on
        self.grid_size.
        """
        # snap curr pos to closest grid point
        x_options = [self.grid_size * ((cx // self.grid_size) + k) for k in [-1, 0, 1]]
        y_options = [self.grid_size * ((cy // self.grid_size) + k) for k in [-1, 0, 1]]

        cx = min(x_options, key=lambda x: abs(cx - x))
        cy = min(y_options, key=lambda y: abs(cy - y))

        # how far are we from the goal right now?
        dist = np.sqrt((delta_x - cx)**2 + (delta_y - cy)**2)
        out = []

        if dist <= self.grid_size*np.sqrt(2):   # within one grid cell of goal
            if not self.check_intersecting(delta_x, delta_y, observation):
                g = np.sqrt((delta_x - ox)**2 + (delta_y - oy)**2)
                return [(dist + prev_cost, curr_state + [(delta_x, delta_y)], dist + prev_cost)]

        options = [-self.grid_size, 0, self.grid_size]  # directions we can move
        for o1 in options:
            for o2 in options:
                if not (o1 == 0 and o2 == 0):   # don't just stay in current grid cell
                    x = o1 + cx
                    y = o2 + cy

                    f = prev_cost + np.sqrt(o1**2 + o2**2)
                    g = np.sqrt((delta_x - x)**2 + (delta_y - y)**2)

                    tot_cost = f + g
                    if not self.check_intersecting(x, y, observation):
                        out.append((tot_cost, curr_state + [(x, y)], f))

        return out


    def a_star(self, start_x, start_y, delta_x, delta_y, observation):
        """
        Perform A* search from a starting x and y to a destination x and y
        while avoiding obstacles seen in the current observation obs.

        start_x and start_y are in meters, with the 'origin' being at
        LatLon(self.start_lat, self.start_lon)
        """
        if self.check_intersecting(delta_x, delta_y, observation):
            print("obstacle too close to target location")
            return [], True

        # Compute how much of the path we planned before is still valid
        valid = []
        changed = False
        for i in range(self.last_subgoal_idx, len(self.path)):
            pt = self.path[i]
            if not self.check_intersecting(pt[0], pt[1], observation):
                valid.append(pt)
            else:
                changed = True
                break

        if not changed and len(self.path) != 0:
            self.replot = False
            return self.path, False

        self.replot = True  # There was an issue w/ prev path, so we will need to draw a new one

        if len(valid) != 0:     # If there is something we can use, start from there
            frontier = self.next_states(delta_x, delta_y, observation, curr_state=valid, cx=valid[-1][0], cy=valid[-1][1])
            visited = set((valid[-1][0], valid[-1][1]))
        else:   # If there is nothing we can use, start from start_x, start_y
            frontier = self.next_states(delta_x, delta_y, observation, curr_state=valid, cx=start_x, cy=start_y)
            visited = set((start_x, start_y))

        heapq.heapify(frontier)     # Make frontier a priority queue

        num_evals = 0

        while len(frontier) > 0 and num_evals < self.max_evals:
            val_new, s_new, f_new = heapq.heappop(frontier)
            pos_new = s_new[-1]

            if pos_new not in visited:
                if pos_new not in self.path:    # Just to print whenever we are deviating from the previous path
                    print(f"pos: {(start_x, start_y)}")
                    print(f"path: {self.path}")
                    print(f"exploring: {pos_new}")

                visited.add(pos_new)

                # We reached the goal location
                if pos_new[0] == delta_x and pos_new[1] == delta_y:
                    return s_new, True

                # We haven't reached the goal, so compute all next states we can go to
                possible = self.next_states(delta_x, delta_y, observation, s_new, prev_cost=f_new, cx=pos_new[0], cy=pos_new[1])

                # If we haven't seen the state before, add it to the queue
                for k in possible:
                    if k[1][-1] not in visited:
                        heapq.heappush(frontier, k)

                num_evals += 1

        if num_evals == self.max_evals:
            print("maxed evals")

        return [], True


    def draw(self, delta_x, delta_y, path, state, path_without=None):
        """
        Plots path and obstacles on matplotlib plot.
        """
        fig, ax = plt.subplots()
        grid_res = self.grid_size

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


    def control(self, boat_angle, delta_x, delta_y, boat_speed, boat_ang_vel, gains):
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

        targ_v = gains * np.matmul(R_inv, eta_d)
        curr_v = np.array([boat_speed, 0, boat_ang_vel]).reshape((3, 1))
        diff_v = targ_v - curr_v

        control = gains * diff_v
        control[2][0] = np.rad2deg(control[2][0])
        control = np.clip(control, np.array([-self.a_max, 0, -self.max_alpha_mag]).reshape(3, 1), np.array([self.a_max, 0, self.max_alpha_mag]).reshape(3, 1))

        dist = np.sqrt((delta_x ** 2) + (delta_y ** 2))

        # print(f"self.running_dist_err: {self.running_dist_err}, self.running_angle_err: {self.running_angle_err}")
        if self.print_info:
            print(f"dist: {round(dist, 5)},  curr_vel: {round(boat_speed, 5)}, accel: {round(control[0][0], 5)}, alpha: {round(control[2][0], 5)}")

        return Action(2, [control[2][0], control[0][0]])


    def compute_gains(self, state, boat_x, boat_y):
        boat_pos = LatLon(self.start_lat, self.start_lon).add_dist(boat_x, boat_y)
        obstacles = state[-1]

        m = 0
        for o in obstacles:
            d = LatLon.dist(boat_pos, LatLon(o[2], o[1]))
            if d < self.clearance + o[0]:
                print(f"too close: {d - o[0]}")
                return (2*self.clearance/(d - o[0]))*self.p_scale
        return self.p_scale


    def select_sub_waypoint(self, path, boat_x, boat_y):
        """
        Select which point on the planned path we should try steering
        the boat to
        """

        if len(path) == 0:
            return -1

        def dist(pt):
            return np.sqrt((pt[0] - boat_x)**2 + (pt[1] - boat_y)**2)

        if dist(path[self.last_subgoal_idx]) < self.grid_size and self.last_subgoal_idx < len(path) - 1:
            return self.last_subgoal_idx + 1

        return self.last_subgoal_idx

        # closest_idx = min(range(0, len(path)), key=lambda x: dist(path[x]))
        # idx = closest_idx
        #
        # while dist(path[idx]) < self.grid_size and idx < len(path) - 1:
        #     idx += 1
        #
        # # idx = min(len(path) - 1, closest_idx + 1)
        # return idx


    def dodge(self, delta_x, delta_y, boat_x, boat_y, state):
        if self.check_intersecting(boat_x, boat_y, state):
            free_states = self.next_states(delta_x, delta_y, state, cx=boat_x, cy=boat_y)
            if len(free_states) > 0:
                return self.next_states(delta_x, delta_y, state, cx=boat_x, cy=boat_y)[0][1][0]
            return (boat_x, boat_y)
        return (boat_x, boat_y)


    # uses ground truth state
    def select_action_from_state(self, env, state):
        """
        Main method that performs A* search, selects subgoal to go to,
        and outputs which actions need to be taken.
        """
        if self.in_sim:
            env.set_waypoint(self.curr_waypoint)

        # if env.total_time < 1:
        #     self.path = []
        #     self.start_lon = None
        #     self.start_lat = None
        #     self.subgoal_idx = 0
        #     self.last_subgoal_idx = 0
        #     return Action(0, 0)

        boat_x, boat_y, boat_speed, _, boat_angle, boat_ang_vel, ocean_current_x, ocean_current_y, obstacles = state

        waypoint = [env.waypoints[self.curr_waypoint].lon, env.waypoints[self.curr_waypoint].lat]

        if self.start_lon is None:
            self.start_lon = boat_x
            self.start_lat = boat_y

        delta_x, delta_y = self.get_distances(waypoint, self.start_lon, self.start_lat)
        boat_x, boat_y = self.get_distances([boat_x, boat_y], self.start_lon, self.start_lat)

        dist = np.sqrt((delta_x - boat_x)**2 + (delta_y - boat_y)**2)

        if dist < 0.05:
            self.curr_waypoint = (self.curr_waypoint + 1) % len(env.waypoints)
            self.path = []
            self.start_lon = None
            self.start_lat = None
            self.subgoal_idx = 0
            self.last_subgoal_idx = 0
            return Action(0, 0)

        # new path and whether it is different from previously planned path
        path_with, changed = self.a_star(boat_x, boat_y, delta_x, delta_y, state)

        if changed:
            self.last_subgoal_idx = 0   # 'restarting' on a new path

        # if the subgoal is too close, select the one after it
        pt_idx = self.select_sub_waypoint(path_with, boat_x, boat_y)
        pt = (boat_x, boat_y)
        if pt_idx != -1:    # path planner could find a path
            self.last_subgoal_idx = pt_idx
            pt = path_with[pt_idx]
            self.path = path_with
        else:   # no path found
            pt = self.dodge(delta_x, delta_y, boat_x, boat_y, state)
            self.last_subgoal_idx = 0
            self.path = []


        # print(f"path: {[self.start_lon, self.start_lat] + self.path}")

        if self.replot and len(path_with) > 0 and self.in_sim:
            path_to_plot = [LatLon(self.start_lat, self.start_lon).add_dist(boat_x, boat_y)] + [LatLon(self.start_lat, self.start_lon).add_dist(p[0], p[1]) for p in path_with[pt_idx:]]
            env.plot_path(path_to_plot)

        return self.control(boat_angle, pt[0] - boat_x, pt[1] - boat_y, boat_speed, boat_ang_vel, gains=self.compute_gains(state, boat_x, boat_y))
