import numpy as np
import pygame

from scipy.optimize import minimize, Bounds, differential_evolution
import scipy.integrate as integrate

from controller.base_controller import BaseController
from boat_simulation.simple import Action
from boat_simulation.latlon import LatLon

# Boat is modelled as a rod with two thrusters on each end
class SLSQPController(BaseController):
    def __init__(self, in_sim=True):
        BaseController.__init__(self, "Minimal controller for autonomy")
        self.in_sim = in_sim

        self.f_max = 50
        self.boat_mass = 5
        self.boat_width = 0.5

        self.a_max = 2 * self.f_max / (self.boat_mass)
        self.max_alpha_mag = 6 * self.f_max / (self.boat_mass * self.boat_width)

        self.curr_waypoint = 0

        self.last_a = 0
        self.last_alpha = 0

        self.last_dist = None
        self.accumulator = 0
        self.a_rate = 50
        self.a_constant = 5e-6


    def compute_objective_theoretical(self, input, currPos, targPos, theta_i, ang_vel, v_i, v_cx, v_cy, t=1):
        a = input[0]
        alpha = input[1]

        def theta(time):
            return theta_i + ang_vel * time + .5 * alpha * (time**2)

        # handle x first
        def delta_x(time):
            return -(v_i + a * time) * np.sin(np.deg2rad(theta(time))) - v_cx

        delta_x_tot = integrate.quad(delta_x, 0, t)[0]

        def delta_y(time):
            return -(v_i + a * time) * np.cos(np.deg2rad(theta(time))) - v_cy

        delta_y_tot = integrate.quad(delta_y, 0, t)[0]

        return LatLon.dist(currPos.add_dist(delta_x_tot, delta_y_tot), targPos) ** 2


    def compute_objective(self, input, currPos, targPos, theta_i, ang_vel, v_i, v_cx, v_cy, t=1):
        a = input[0]
        alpha = input[1]

        theta_final = theta_i + ang_vel * t + .5 * alpha * (t**2)

        x_curr, y_curr = currPos.lon, currPos.lat
        x_targ, y_targ = targPos.lon, targPos.lat

        delta_x = LatLon.dist(LatLon(y_targ, x_curr), LatLon(y_targ, x_targ))
        if x_targ > x_curr:
            delta_x *= -1

        delta_y = LatLon.dist(LatLon(y_curr, x_targ), LatLon(y_targ, x_targ))
        if y_targ > y_curr:
            delta_y *= -1

        delta_x_tot = -(0.5*a*t**2 + v_i*t)*np.sin(np.deg2rad(theta_final)) + v_cx*t
        delta_y_tot = -(0.5*a*t**2 + v_i*t)*np.cos(np.deg2rad(theta_final)) + v_cy*t

        return (delta_x + delta_x_tot)**2 + (delta_y + delta_y_tot)**2
        # return 100 * LatLon.dist(currPos.add_dist(delta_x_tot, delta_y_tot), targPos)


    def initial_control(self, theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy, t):
        return (self.last_a, self.last_alpha)


    def new_control(self, theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy):
        currPos = LatLon(y_curr, x_curr)
        targPos = LatLon(y_targ, x_targ)

        dist = LatLon.dist(currPos, targPos)

        obj_fun = self.compute_objective
        bounds = Bounds([-self.a_max, -self.max_alpha_mag], [self.a_max, self.max_alpha_mag])

        t = 1.0
        delta = 0
        penalty = 0

        if self.last_dist is not None:
            delta = abs(dist) - abs(self.last_dist)
            self.accumulator += np.exp(-1600 * delta**2) * self.a_rate
            penalty = self.a_constant * self.accumulator

        t = max(t - penalty, 1e-3)

        guess = self.initial_control(theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy, t)
        guess = np.array([self.last_a, self.last_alpha])

        solved = minimize(obj_fun, guess,
            (currPos, targPos, theta_i, ang_vel, v_i, v_cx, v_cy, t),
            method='slsqp', bounds=bounds,
            options={'maxiter': 200})

        out = solved.x

        # out = guess

        print(f"dist: {round(LatLon.dist(currPos, targPos), 5)},  curr_vel: {round(v_i, 5)}, accel: {round(out[0], 5)}, alpha: {round(out[1], 5)}, init alpha: {guess[1]}, t: {round(t, 5)}")

        return out


    def estimate_currents(self, env, state):
        boat_x, boat_y = state[0], state[1]
        ocean_current_x, ocean_current_y = env.compute_ocean_current(LatLon(boat_y, boat_x))

        return ocean_current_x, ocean_current_y


    # uses ground truth state
    def select_action_from_state(self, env, state):
        if self.in_sim:
            env.set_waypoint(self.curr_waypoint)

        if env.total_time < 1:
            return Action(0, 0)

        boat_x, boat_y, boat_speed, boat_angle, boat_ang_vel, obstacles = state
        boat_speed = env.speed

        waypoint = [env.waypoints[self.curr_waypoint].lon, env.waypoints[self.curr_waypoint].lat]
        dist = LatLon.dist(env.boat_coords, env.waypoints[self.curr_waypoint])

        if abs(dist) < 0.05:
            self.curr_waypoint = (self.curr_waypoint + 1) % len(env.waypoints)
            self.last_dist = None
            self.accumulator = 0

        ocean_current_x, ocean_current_y = self.estimate_currents(env, state)

        control = self.new_control(boat_angle, boat_ang_vel, waypoint[0], boat_x, waypoint[1], boat_y, boat_speed, ocean_current_x, ocean_current_y)

        self.last_a = control[0]
        self.last_alpha = control[1]

        self.last_dist = dist

        return Action(2, [control[1], control[0]])
