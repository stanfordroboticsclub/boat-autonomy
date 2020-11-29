import numpy as np
import pygame

from scipy.optimize import minimize, Bounds
import scipy.integrate as integrate

from controller.base_controller import BaseController
from boat_simulation.simple import Action, latlon_to_xy, PIXELS_PER_METER
from boat_simulation.latlon import LatLon

VEL_SCALE = 1/60

# Boat is modelled as a rod with two thrusters on each end
class ScipyOptController(BaseController):
    def __init__(self, in_sim=True):
        BaseController.__init__(self, "Minimal controller for autonomy")
        self.in_sim = in_sim

        # 5 kg f ~ 50 N
        # https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster/
        self.f_max = 50
        self.boat_mass = 5
        self.boat_width = 1

        self.a_max = 2 * self.f_max / self.boat_mass
        self.max_alpha_mag = 6 * self.f_max / (self.boat_mass * self.boat_width)

        print(f"A MAX: {self.a_max}")
        print(f"ALPHA MAX: {self.max_alpha_mag}")

        self.accelerated = 50
        self.running_error = 0
        self.i_constant = 5e-6

        self.last_dist = None

        self.accumulator = 0
        self.a_constant = 5e-6
        self.a_rate = 50

        self.curr_waypoint = 0

        self.last_accel = None
        self.last_alpha = None


    def compute_objective_logging(self, input, theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy, t=1):
        t = max(t - self.i_constant * self.running_error, 1e-3)

        a = input[0]
        alpha = input[1]

        theta = theta_i + ang_vel * t + .5 * alpha * (t**2)

        # delta_x = x_targ - x_curr

        delta_x = LatLon.dist(LatLon(y_curr, x_curr), LatLon(y_curr, x_targ))

        if x_targ < x_curr:
            delta_x *= -1

        dx_vel = (v_i * t + .5 * a *( t ** 2)) * np.sin(np.deg2rad(theta))
        dx_curr = v_cx * t

        dx_total = delta_x + dx_vel - dx_curr

        # delta_y = y_targ - y_curr

        delta_y = LatLon.dist(LatLon(y_curr, x_curr), LatLon(y_targ, x_curr))

        if y_targ < y_curr:
            delta_y *= -1

        dy_vel = (v_i * t + .5 * a * (t ** 2)) * np.cos(np.deg2rad(theta))
        dy_curr = v_cy * t

        dy_total = delta_y + dy_vel - dy_curr

        return (dx_total) ** 2 + (dy_total) ** 2


    def compute_objective_latlon(self, input, theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy, t=1):
        a = input[0]
        alpha = input[1]

        theta = theta_i + ang_vel * t + .5 * alpha * (t**2)

        # delta_x = x_targ - x_curr
        dx_vel = -(v_i * t + .5 * a *( t ** 2)) * np.sin(np.deg2rad(theta))
        dx_curr = -v_cx * t

        dx_total = dx_vel - dx_curr

        # delta_y = y_targ - y_curr
        dy_vel = -(v_i * t + .5 * a * (t ** 2)) * np.cos(np.deg2rad(theta))
        dy_curr = -v_cy * t

        dy_total = dy_vel - dy_curr

        out = LatLon.dist(LatLon(y_targ, x_targ), LatLon(y_curr, x_curr).add_dist(dx_total, dy_total))

        return out


    def compute_objective_integrated(self, input, theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy, t=1):
        a = input[0]
        alpha = input[1]

        theta = theta_i + ang_vel * t + .5 * alpha * (t**2)
        theta_deg = np.deg2rad(theta_i)

        def theta(time):
            return theta_i + ang_vel * time + .5 * alpha * (time**2)

        # handle x first
        def delta_x(time):
            return -(v_i*time + 0.5 * a * time**2) * np.sin(np.deg2rad(theta(time)))

        delta_x_tot = integrate.quad(delta_x, 0, t)[0]

        def delta_y(time):
            return -(v_i*time + 0.5 * a * time**2) * np.cos(np.deg2rad(theta(time)))

        delta_y_tot = integrate.quad(delta_y, 0, t)[0]

        return LatLon.dist(LatLon(y_curr, x_curr).add_dist(delta_x_tot, delta_y_tot), LatLon(y_targ, x_targ)) + 5e-2*np.abs(ang_vel + alpha) + 5e-2*np.abs(v_i + a)

        # # handle x first
        # d_vcx = -.5 * v_cx * t**2
        #
        # # v_i*t term of integral
        # x_vit_first = 0.5 * v_i * np.sin(theta_deg) * t**2
        # x_vit_second = (v_i * ang_vel * np.cos(theta_deg) * t**3) / 3
        # x_vit_third = (0.5 * (-(ang_vel**2)*np.sin(theta_deg) + alpha * np.cos(theta_deg)) * v_i * t**4) / 4
        # x_vit_tot = -x_vit_first - x_vit_second - x_vit_third
        #
        # # .5 a t^2 term of integral
        # x_hats_first = (0.5 * a * np.sin(theta_deg) * t**3) / 3
        # x_hats_second = (0.5 * a * ang_vel * np.cos(theta_deg) * t**4) / 4
        # x_hats_third = (0.25 * (-(ang_vel**2)*np.sin(theta_deg) + alpha * np.cos(theta_deg)) * a * t**5) / 5
        # x_hats_tot = -x_hats_first - x_hats_second - x_hats_third
        #
        # delta_x = d_vcx + x_vit_tot + x_hats_tot
        #
        # # handle y
        # d_vcy = -.5 * v_cy * t**2
        #
        # y_vit_first = 0.5 * v_i * np.cos(theta_deg) * t**2
        # y_vit_second = -(v_i * ang_vel * np.sin(theta_deg) * t**3) / 3
        # y_vit_third = (0.5 * (-(ang_vel**2)*np.cos(theta_deg) - alpha * np.sin(theta_deg)) * v_i * t**4) / 4
        # y_vit_tot = -y_vit_first - y_vit_second - y_vit_third
        #
        # y_hats_first = (0.5 * a * np.cos(theta_deg) * t**3) / 3
        # y_hats_second = -(0.5 * a * ang_vel * np.sin(theta_deg) * t**4) / 4
        # y_hats_third = (0.25 * (-(ang_vel**2)*np.cos(theta_deg) - alpha * np.sin(theta_deg)) * a * t**5) / 5
        # y_hats_tot = -y_hats_first - y_hats_second
        #
        # delta_y = d_vcy + y_vit_tot + y_hats_tot

        # return LatLon.dist(LatLon(y_curr, x_curr).add_dist(delta_x, delta_y), LatLon(y_targ, x_targ))


    def compute_objective_xy(self, input, theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy, t=1):
        # t = max(t - self.i_constant * self.running_error, 1e-3)

        a = input[0]
        alpha = input[1]

        theta = theta_i + ang_vel * t + .5 * alpha * (t**2)

        x_targ, y_targ = latlon_to_xy(LatLon(y_targ, x_targ))
        x_curr, y_curr = latlon_to_xy(LatLon(y_curr, x_curr))

        delta_x = (x_targ - x_curr) / PIXELS_PER_METER
        dx_vel = (v_i * t + .5 * a *( t ** 2)) * np.sin(np.deg2rad(theta))
        dx_curr = v_cx * t

        dx_total = delta_x + dx_vel - dx_curr

        delta_y = (y_targ - y_curr) / PIXELS_PER_METER
        dy_vel = (v_i * t + .5 * a * (t ** 2)) * np.cos(np.deg2rad(theta))
        dy_curr = v_cy * t

        dy_total = delta_y + dy_vel - dy_curr

        return (dx_total) ** 2 + (dy_total) ** 2


    def compute_objective(self, input, theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy, t=1):
        params = (input, theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy, t)
        # return self.compute_objective_logging(*params)
        return self.compute_objective_integrated(*params)


    def new_control(self, theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy, use_accumulator=True):
        obj_fun = self.compute_objective

        dist = LatLon.dist(LatLon(y_curr, x_curr), LatLon(y_targ, x_targ))
        angle = np.arctan2(x_curr - x_targ, y_curr - y_targ) * 180 / np.pi

        # alpha_init = self.compute_angular_accel(ang_vel, theta_i, angle)
        # accel_init = self.compute_accel(dist, v_i, theta_i, angle)

        if self.last_alpha is None:
            alpha_init = self.compute_angular_accel(ang_vel, theta_i, angle)
            accel_init = self.compute_accel(dist, v_i, theta_i, angle)
        else:
            alpha_init = self.last_alpha
            accel_init = self.last_accel

        bounds = Bounds([-self.a_max, -self.max_alpha_mag], [self.a_max, self.max_alpha_mag])

        t = 1
        delta = 0
        penalty = self.i_constant * self.running_error

        if use_accumulator and self.last_dist is not None:
            delta = np.abs(dist) - np.abs(self.last_dist)
            self.accumulator += np.exp(-100 * delta**2) * self.a_rate

            penalty = self.a_constant * self.accumulator

        t = max(t - penalty, 1e-3)

        solved = minimize(obj_fun, np.array([accel_init, alpha_init]),
            (theta_i, ang_vel, x_targ, x_curr, y_targ, y_curr, v_i, v_cx, v_cy, t), method='trust-constr', bounds=bounds,
            options={'maxiter': 3})

        x = solved.x

        print(f"dist: {round(dist, 5)},  curr_vel: {round(v_i, 5)}, accel: {round(x[0], 5)}, alpha: {round(x[1], 5)}, t: {round(t, 5)}, delta: {round(delta, 5)}")

        solved = [np.clip(x[0], -self.a_max, self.a_max), np.clip(x[1], -self.max_alpha_mag, self.max_alpha_mag)]

        # print(f"dist: {round(dist, 5)},  curr_vel: {round(v_i, 5)}, accel: {round(solved[0], 5)}, alpha: {round(solved[1], 5)}, t: {round(t, 5)}, delta: {round(delta, 5)}")
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


    def estimate_currents(self, env, state):
        # a = 1
        # b = -(2*ocean_current_x*np.sin(np.deg2rad(boat_angle)) + 2*ocean_current_y*np.cos(np.deg2rad(boat_angle)))
        # c = ocean_current_x**2 + ocean_current_y**2 - (VEL_SCALE**2)*(boat_speed**2)
        #
        # boat_dx = VEL_SCALE * boat_speed * np.sin(np.pi * boat_angle / 180)
        # boat_dy = VEL_SCALE * boat_speed * np.cos(np.pi * boat_angle / 180)
        #
        # ocean_current_x /= VEL_SCALE
        # ocean_current_y /= VEL_SCALE
        #
        # # print(b**2 - 4*a*c)
        # boat_speed = (-b + np.sqrt(max(b**2 - 4*a*c, 0))) / (2*a)
        # boat_speed /= VEL_SCALE
        #
        # intended_boat_dx = VEL_SCALE * boat_speed * np.sin(np.pi * boat_angle / 180)
        # intended_boat_dy = VEL_SCALE * boat_speed * np.cos(np.pi * boat_angle / 180)
        #
        # projection = (intended_boat_dx * boat_dx + intended_boat_dy * boat_dy) / (VEL_SCALE * boat_speed)
        # if projection < 0:
        #     boat_speed *= -1

        boat_lon, boat_lat = state[0], state[1]
        ocean_current_x, ocean_current_y = env.compute_ocean_current(LatLon(boat_lat, boat_lon))

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
            self.running_error = 0
            self.accumulator = 0
            self.last_dist = 0

        self.running_error += abs(dist)

        ocean_current_x, ocean_current_y = self.estimate_currents(env, state)

        control = self.new_control(boat_angle, boat_ang_vel, waypoint[0], boat_x, waypoint[1], boat_y, boat_speed, ocean_current_x, ocean_current_y, True)
        self.last_dist = dist

        self.last_alpha = control[1]
        self.last_accel = control[0]

        return Action(2, [control[1], control[0]])
