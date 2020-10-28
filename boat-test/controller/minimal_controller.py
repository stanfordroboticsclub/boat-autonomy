import numpy as np

from controller.base_controller import BaseController
from boat_simulation.simple import Action


# Boat is modelled as a rod with two thrusters on each end
class MinimalController(BaseController):
    def __init__(self):
        BaseController.__init__(self, "Minimal controller for autonomy")
        self.curr_waypoint = 0  # 0 is assumed to be the boat's current position

        self.f_max = 5
        self.boat_mass = 5
        self.boat_width = 1

        self.a_max = 2 * self.f_max / self.boat_mass
        self.max_alpha_mag = 3 * self.f_max / (self.boat_mass * self.boat_width)

        self.accelerated = 50


    def compute_angular_accel(self, ang_vel, curr_heading, target_heading, max_t=1/2):
        alpha_s = -ang_vel / max_t                          # make stationary in max_t steps
        alpha_0 = -2*curr_heading / (max_t ** 2)            # make heading = 0
        alpha_heading = 2*target_heading / (max_t ** 2)     # turn to target heading

        alpha = alpha_s + alpha_0 + alpha_heading
        return np.clip(alpha, -self.max_alpha_mag, self.max_alpha_mag)


    def compute_accel(self, dist, curr_vel, curr_heading, target_heading, max_t=1):
        if dist < 1:
            return -curr_vel

        dist = np.cos(np.deg2rad(target_heading - curr_heading)) * dist
        accel = np.clip(2*(dist - curr_vel*max_t) / (max_t**2), -self.a_max, self.a_max)

        print(f"dist: {round(dist, 5)},  curr_vel: {round(curr_vel, 5)},  max t: {round(max_t, 5)},  accel: {round(accel, 5)}")
        return accel


    # uses ground truth state
    def select_action_from_state(self, env, state):

        if env.total_time < 1:
            # return Action(0, self.a_max)
            return Action(0, 0)

        boat_x, boat_y, boat_speed, boat_angle, boat_ang_vel, obstacles = state
        waypoint = env.waypoints[self.curr_waypoint]

        dist = np.sqrt((boat_x - waypoint[0]) ** 2 + (boat_y - waypoint[1]) ** 2)

        if dist < 2:
            self.curr_waypoint = (self.curr_waypoint + 1) % len(env.waypoints)

        angle = np.arctan2(boat_x - waypoint[0], boat_y - waypoint[1]) * 180 / np.pi

        alpha = self.compute_angular_accel(boat_ang_vel, boat_angle, angle)
        accel = self.compute_accel(dist, boat_speed, boat_angle, angle)

        return Action(2, [alpha, accel])
