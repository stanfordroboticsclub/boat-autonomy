import numpy as np
import pygame

from scipy.optimize import minimize, Bounds, differential_evolution
import scipy.integrate as integrate

from controller.base_controller import BaseController
from boat_simulation.simple import Action, latlon_to_xy, PIXELS_PER_METER
from boat_simulation.latlon import LatLon

VEL_SCALE = 1/60
PIXELS_PER_METER = 30

# Boat is modelled as a rod with two thrusters on each end
class PIDController(BaseController):
    def __init__(self, in_sim=True):
        BaseController.__init__(self, "Minimal controller for autonomy")
        self.in_sim = in_sim

        self.f_max = 50
        self.boat_mass = 5
        self.boat_width = 0.5

        self.a_max = 2 * self.f_max / (self.boat_mass)
        self.max_alpha_mag = 6 * self.f_max / (self.boat_mass * self.boat_width)

        self.curr_waypoint = 0

        self.p_scale = np.array([1, 0, 1]).reshape(3, 1)
        self.i_scale = np.array([5e-4, 0, 5e-4]).reshape(3, 1)

        self.last_dist = None
        self.last_angle = None

        self.running_dist_err = 0
        self.running_angle_err = 0


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


    # uses ground truth state
    def select_action_from_state(self, env, state):
        if self.in_sim:
            env.set_waypoint(self.curr_waypoint)

        if env.total_time < 1:
            return Action(0, 0)

        boat_x, boat_y, boat_speed, _, boat_angle, boat_ang_vel, ocean_current_x, ocean_current_y, obstacles = state

        waypoint = [env.waypoints[self.curr_waypoint].lon, env.waypoints[self.curr_waypoint].lat]
        dist = LatLon.dist(LatLon(boat_y, boat_x), LatLon(waypoint[1], waypoint[0]))

        if abs(dist) < 0.05:
            self.curr_waypoint = (self.curr_waypoint + 1) % len(env.waypoints)
            self.running_dist_err = 0
            self.running_angle_err = 0

        boat_angle_deg = np.deg2rad(boat_angle)
        R = np.array([  [-np.sin(boat_angle_deg),   np.cos(boat_angle_deg) ,    0],
                        [-np.cos(boat_angle_deg),  -np.sin(boat_angle_deg),    0],
                        [0,                         0,                          1]]).reshape((3, 3))
        R_inv = R.T

        delta_x, delta_y = self.get_distances(waypoint, boat_x, boat_y)
        angle = self.get_required_angle_change(boat_angle, delta_x, delta_y)

        err_vec = np.array([self.running_dist_err, 0, self.running_angle_err])

        # print(err_vec)

        eta_d = np.array([delta_x, delta_y, angle]).reshape((3, 1))

        targ_v = (self.p_scale * np.matmul(R_inv, eta_d)) + (self.i_scale * err_vec)
        curr_v = np.array([boat_speed, 0, boat_ang_vel]).reshape((3, 1))
        diff_v = targ_v - curr_v

        control = self.p_scale * diff_v
        control[2][0] = np.rad2deg(control[2][0])
        control = np.clip(control, np.array([-self.a_max, 0, -self.max_alpha_mag]).reshape(3, 1), np.array([self.a_max, 0, self.max_alpha_mag]).reshape(3, 1))

        if self.last_dist is not None and self.last_angle is not None:
            delta = abs(dist) - abs(self.last_dist)
            delta_angle = abs(angle) - abs(self.last_angle)

            d_increment = np.exp(-1600 * delta**2) * dist
            a_increment = np.exp(-1600 * delta_angle**2) * angle

            raw_angle = (np.arctan2(-delta_x, -delta_y) * 180 / np.pi) - (boat_angle)
            raw_angle %= 360
            raw_angle = min(raw_angle, raw_angle - 360, key=abs)

            if abs(raw_angle) < 90:
                self.running_dist_err += d_increment
            else:
                self.running_dist_err -= d_increment
            self.running_angle_err += a_increment

        self.last_dist = dist
        self.last_angle = angle

        # print(f"self.running_dist_err: {self.running_dist_err}, self.running_angle_err: {self.running_angle_err}")
        print(f"dist: {round(dist, 5)},  curr_vel: {round(boat_speed, 5)}, accel: {round(control[0][0], 5)}, alpha: {round(control[2][0], 5)}")

        return Action(2, [control[2][0], control[0][0]])
