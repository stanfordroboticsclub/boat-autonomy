import numpy as np
import pygame

from controller.base_controller import BaseController
from boat_simulation.simple import Action
from boat_simulation.latlon import LatLon


# Boat is modelled as a rod with two thrusters on each end
class MinimalController(BaseController):
    def __init__(self, in_sim=True):
        BaseController.__init__(self, "minimal", in_sim)
        self.in_sim = in_sim

        # 5 kg f ~ 50 N
        # https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster/
        self.f_max = 50
        self.boat_mass = 5
        self.boat_width = 1

        self.a_max = 2 * self.f_max / self.boat_mass
        self.max_alpha_mag = 6 * self.f_max / (self.boat_mass * self.boat_width)

        self.accelerated = 50
        self.running_error = 0
        self.i_constant = 4e-6

        self.curr_waypoint = 0


    def compute_angular_accel(self, ang_vel, curr_heading, target_heading, max_t=1):
        alpha_s = -ang_vel / max_t                          # make stationary in max_t steps
        alpha_0 = -2*curr_heading / (max_t ** 2)            # make heading = 0
        alpha_heading = 2*target_heading / (max_t ** 2)     # turn to target heading

        alpha = alpha_s + alpha_0 + alpha_heading
        return np.clip(alpha, -self.max_alpha_mag, self.max_alpha_mag)


    def compute_accel(self, dist, curr_vel, curr_heading, target_heading, max_t=1):
        if dist < 0.01:
            return -curr_vel

        max_t = max(max_t - self.i_constant * self.running_error, 1e-3)

        if max_t == 1e-3:
            self.running_error = 0

        dist = np.cos(np.deg2rad(target_heading - curr_heading)) * dist
        accel = np.clip(2*(dist - curr_vel*max_t) / (max_t**2), -self.a_max, self.a_max)

        print(f"dist: {round(dist, 5)},  curr_vel: {round(curr_vel, 5)},  max t: {round(max_t, 5)},  accel: {round(accel, 5)}, running_error: {round(self.running_error, 5)}")
        return accel


    # uses ground truth state
    def select_action_from_state(self, env, state):

        if self.in_sim:
            env.set_waypoint(self.curr_waypoint)

        # if env.total_time < 1:
        #     # return Action(0, self.a_max)
        #     return Action(0, 0)

        boat_x, boat_y, boat_speed, desired_speed, boat_angle, boat_ang_vel, ocean_current_x, ocean_current_y, obstacles = state
        waypoint = [env.waypoints[self.curr_waypoint].lon, env.waypoints[self.curr_waypoint].lat]
        dist = LatLon.dist(LatLon(boat_y, boat_x), LatLon(waypoint[1], waypoint[0]))

        if abs(dist) < 0.05 and abs(boat_speed) < 5:
            self.curr_waypoint = (self.curr_waypoint + 1) % len(env.waypoints)
            self.running_error = 0

        self.running_error += abs(dist)

        angle = np.arctan2(boat_x - waypoint[0], boat_y - waypoint[1]) * 180 / np.pi

        alpha = self.compute_angular_accel(boat_ang_vel, boat_angle, angle)
        accel = self.compute_accel(dist, boat_speed, boat_angle, angle)

        return Action(2, [alpha, accel])
