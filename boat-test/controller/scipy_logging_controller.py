import numpy as np
import pygame
import pandas as pd
import sys

from scipy.optimize import minimize, Bounds

from controller.base_controller import BaseController
from boat_simulation.simple import Action


# Boat is modelled as a rod with two thrusters on each end
class ScipyLoggingController(BaseController):
    def __init__(self, in_sim=True):
        BaseController.__init__(self, "Minimal controller for autonomy")
        self.in_sim = in_sim

        self.f_max = 5
        self.boat_mass = 5
        self.boat_width = 1

        self.a_max = 2 * self.f_max / self.boat_mass
        self.max_alpha_mag = 3 * self.f_max / (self.boat_mass * self.boat_width)

        self.accelerated = 50
        self.running_error = 0
        self.i_constant = 5e-6

        self.curr_waypoint = 0
        self.df = pd.DataFrame(columns="theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy, accel_init, alpha_init, accel_solved, alpha_solved".split(', '))


    def compute_objective(self, input, theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy, t=1):
        t = max(t - self.i_constant * self.running_error, 1e-3)

        a = input[0]
        alpha = input[1]

        theta = theta_i + ang_vel * t + .5 * alpha * (t**2)

        delta_x = x_targ - x_curr
        dx_vel = (v_i * t + .5 * a *( t ** 2)) * np.sin(np.deg2rad(theta))
        dx_curr = v_cx * t

        dx_total = delta_x + dx_vel - dx_curr

        delta_y = y_targ - y_curr
        dy_vel = (v_i * t + .5 * a * (t ** 2)) * np.cos(np.deg2rad(theta))
        dy_curr = v_cy * t

        dy_total = delta_y + dy_vel - dy_curr

        return (dx_total) ** 2 + (dy_total) ** 2


    def new_control(self, theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy):
        # obj_fun = self.compute_objective_func(theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy)
        obj_fun = self.compute_objective

        dist = np.sqrt((x_curr - x_targ) ** 2 + (y_curr - y_targ) ** 2)
        angle = np.arctan2(x_curr - x_targ, y_curr - y_targ) * 180 / np.pi

        alpha_init = self.compute_angular_accel(ang_vel, theta_i, angle)
        accel_init = self.compute_accel(dist, v_i, theta_i, angle)

        # print("before dist")
        # print(self.compute_objective(np.array([accel_init, alpha_init]),
        #     theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy))

        bounds = Bounds([-self.a_max, -self.max_alpha_mag], [self.a_max, self.max_alpha_mag])

        solved = minimize(obj_fun, np.array([accel_init, alpha_init]),
            (theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy), method='trust-constr', bounds=bounds,
            options={'maxiter': 3})

        # print("solved fun")
        # print(solved.fun)

        x = solved.x
        # print(f"before accel: {x[0]}, before alpha: {x[1]}")
        solved = [np.clip(x[0], -self.a_max, self.a_max), np.clip(x[1], -self.max_alpha_mag, self.max_alpha_mag)]

        print(f"dist: {round(dist, 5)},  curr_vel: {round(v_i, 5)}, accel: {round(solved[0], 5)}, alpha: {round(solved[1], 5)}, running_error: {round(self.running_error, 5)}")

        logging_dict = dict(zip("theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy".split(', '), [theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy]))
        logging_dict["accel_init"] = accel_init
        logging_dict["accel_init"] = alpha_init
        logging_dict["accel_solved"] = solved[0]
        logging_dict["alpha_solved"] = solved[1]
        self.df = self.df.append(logging_dict, ignore_index=True)

        return solved  # (accel, alpha)


    def compute_angular_accel(self, ang_vel, curr_heading, target_heading, max_t=1):
        alpha_s = -ang_vel / max_t                          # make stationary in max_t steps
        alpha_0 = -2*curr_heading / (max_t ** 2)            # make heading = 0
        alpha_heading = 2*target_heading / (max_t ** 2)     # turn to target heading

        alpha = alpha_s + alpha_0 + alpha_heading
        return np.clip(alpha, -self.max_alpha_mag, self.max_alpha_mag)


    def compute_accel(self, dist, curr_vel, curr_heading, target_heading, max_t=1):
        # if dist < 1:
        #     return -curr_vel

        max_t = max(max_t - self.i_constant * self.running_error, 1e-3)

        if max_t == 1e-3:
            self.running_error = 0

        dist = np.cos(np.deg2rad(target_heading - curr_heading)) * dist
        accel = np.clip(2*(dist - curr_vel*max_t) / (max_t**2), -self.a_max, self.a_max)

        # print(f"dist: {round(dist, 5)},  curr_vel: {round(curr_vel, 5)},  max t: {round(max_t, 5)},  accel: {round(accel, 5)}, running_error: {round(self.running_error, 5)}")
        return accel


    # uses ground truth state
    def select_action_from_state(self, env, state):

        if self.in_sim:
            env.set_waypoint(self.curr_waypoint)

        if env.total_time < 1:
            # return Action(0, self.a_max)
            return Action(0, 0)

        boat_x, boat_y, boat_speed, boat_angle, boat_ang_vel, obstacles = state
        waypoint = env.waypoints[self.curr_waypoint]

        dist = np.sqrt((boat_x - waypoint[0]) ** 2 + (boat_y - waypoint[1]) ** 2)
        #
        if abs(dist) < 2:
            self.curr_waypoint = (self.curr_waypoint + 1) % len(env.waypoints)
            self.running_error = 0
            self.df.to_csv("logs/log.csv")
            sys.exit(0)
        #
        self.running_error += abs(dist)
        #
        # angle = np.arctan2(boat_x - waypoint[0], boat_y - waypoint[1]) * 180 / np.pi
        #
        # alpha = self.compute_angular_accel(boat_ang_vel, boat_angle, angle)
        # accel = self.compute_accel(dist, boat_speed, boat_angle, angle)

        # theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy

        ocean_current_x, ocean_current_y = env.compute_ocean_current(boat_x, boat_y)
        ocean_current_x *= 60
        ocean_current_y *= 60

        # ocean_current_x = 0
        # ocean_current_y = 0

        control = self.new_control(boat_angle, boat_ang_vel, waypoint[0], boat_x, waypoint[1], boat_y, boat_speed, ocean_current_x, ocean_current_y)
        # print("control: " + str(control))

        return Action(2, [control[1], control[0]])
