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
        BaseController.__init__(self, "Minimal controller for autonomy", in_sim)
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

        self.vel_penalty = 7.5e-4
        self.ang_vel_penalty = 7.5e-4

        self.last_state = None


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

        final_ang_vel = ang_vel + alpha*t
        final_speed = v_i + a * t

        return LatLon.dist(currPos.add_dist(delta_x_tot, delta_y_tot), targPos)**2 + self.ang_vel_penalty*np.abs(final_ang_vel) + self.vel_penalty*np.abs(final_speed)


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

        final_ang_vel = ang_vel + alpha*t
        final_speed = v_i + a * t

        return (delta_x + delta_x_tot)**2 + (delta_y + delta_y_tot)**2 + self.ang_vel_penalty*np.abs(final_ang_vel) + self.vel_penalty*np.abs(final_speed)
        # return 100 * LatLon.dist(currPos.add_dist(delta_x_tot, delta_y_tot), targPos)


    def initial_control(self, theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy, t):
        return (self.last_a, self.last_alpha)


    def new_control(self, theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy, print_info=True):
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
        if print_info:
            print(f"dist: {round(LatLon.dist(currPos, targPos), 5)},  curr_vel: {round(v_i, 5)}, accel: {round(out[0], 5)}, alpha: {round(out[1], 5)}, init alpha: {guess[1]}, t: {round(t, 5)}")

        return out


    def estimate_currents_theoretical(self, state, print_current_info=False):
        if self.last_state is None:
            return 0, 0

        boat_x, boat_y, _, boat_speed, boat_angle, boat_ang_vel, ocean_current_x, ocean_current_y, obstacles = state
        old_boat_x, old_boat_y, _, old_boat_speed, old_boat_angle, old_boat_ang_vel, old_ocean_current_x, old_ocean_current_y, old_obstacles = self.last_state

        # where should we have gone?
        t = 1/60

        def theta(time):
            return old_boat_angle + old_boat_ang_vel * time + .5 * self.last_alpha * (time**2)

        # handle x first
        def delta_x(time):
            return -(old_boat_speed + self.last_a * time) * np.sin(np.deg2rad(theta(time)))

        delta_x_tot = integrate.quad(delta_x, 0, t)[0]

        def delta_y(time):
            return -(old_boat_speed + self.last_a * time) * np.cos(np.deg2rad(theta(time)))

        delta_y_tot = integrate.quad(delta_y, 0, t)[0]

        predicted = LatLon(old_boat_y, old_boat_x).add_dist(delta_x_tot, delta_y_tot)
        err_x = LatLon.dist(predicted, LatLon(predicted.lat, boat_x))

        if predicted.lon > boat_x:
            err_x *= -1

        err_y = LatLon.dist(predicted, LatLon(boat_y, predicted.lon))

        if predicted.lat > boat_y:
            err_y *= -1

        curr_x, curr_y = err_x / t, err_y / t
        # curr_x, curr_y = 0 / t, 0 / t

        if print_current_info:
            print(f"estimate: {(curr_x, curr_y)}, truth: {(ocean_current_x, ocean_current_y)}, err: {(100*(curr_x - ocean_current_x), 100*(curr_y - ocean_current_y))}")

        return curr_x, curr_y


    def estimate_currents(self, state, print_current_info=False):
        if self.last_state is None:
            return 0, 0

        boat_x, boat_y, _, boat_speed, boat_angle, boat_ang_vel, ocean_current_x, ocean_current_y, obstacles = state
        old_boat_x, old_boat_y, _, old_boat_speed, old_boat_angle, old_boat_ang_vel, old_ocean_current_x, old_ocean_current_y, old_obstacles = self.last_state

        # where should we have gone?
        t = 1/60

        theta_final = old_boat_angle + old_boat_ang_vel * t + .5 * self.last_alpha * (t**2)

        delta_x_tot = -(0.5*self.last_a*t**2 + old_boat_speed*t)*np.sin(np.deg2rad(theta_final))
        delta_y_tot = -(0.5*self.last_a*t**2 + old_boat_speed*t)*np.cos(np.deg2rad(theta_final))

        predicted = LatLon(old_boat_y, old_boat_x).add_dist(delta_x_tot, delta_y_tot)
        err_x = LatLon.dist(predicted, LatLon(predicted.lat, boat_x))

        if predicted.lon > boat_x:
            err_x *= -1

        err_y = LatLon.dist(predicted, LatLon(boat_y, predicted.lon))

        if predicted.lat > boat_y:
            err_y *= -1

        curr_x, curr_y = err_x / t, err_y / t
        # curr_x, curr_y = 0 / t, 0 / t

        if print_current_info:
            print(f"estimate: {(curr_x, curr_y)}, truth: {(ocean_current_x, ocean_current_y)}, err: {(100*(curr_x - ocean_current_x), 100*(curr_y - ocean_current_y))}")

        return curr_x, curr_y


    # uses ground truth state
    def select_action_from_state(self, env, state):
        if self.in_sim:
            env.set_waypoint(self.curr_waypoint)

        # if env.total_time < 1:
        #     return Action(0, 0)

        boat_x, boat_y, boat_speed, _, boat_angle, boat_ang_vel, ocean_current_x, ocean_current_y, obstacles = state
        ocean_current_x, ocean_current_y = self.estimate_currents_theoretical(state)

        waypoint = [env.waypoints[self.curr_waypoint].lon, env.waypoints[self.curr_waypoint].lat]
        dist = LatLon.dist(LatLon(boat_y, boat_x), LatLon(waypoint[1], waypoint[0]))

        if abs(dist) < 0.05:
            self.curr_waypoint = (self.curr_waypoint + 1) % len(env.waypoints)
            self.last_dist = None
            self.accumulator = 0

        control = self.new_control(boat_angle, boat_ang_vel, waypoint[0], boat_x, waypoint[1], boat_y, boat_speed, ocean_current_x, ocean_current_y)

        control[0] = np.clip(control[0], -self.a_max, self.a_max)
        control[1] = np.clip(control[1], -self.max_alpha_mag, self.max_alpha_mag)

        self.last_a = control[0]
        self.last_alpha = control[1]

        self.last_dist = dist

        self.last_state = state

        return Action(2, [control[1], control[0]])
