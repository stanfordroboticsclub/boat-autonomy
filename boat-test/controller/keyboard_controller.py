import pygame
from controller.base_controller import BaseController
from boat_simulation.simple import Action
from boat_simulation.latlon import LatLon
import numpy as np

from pygame.locals import (
    K_UP,
    K_DOWN,
    K_LEFT,
    K_RIGHT,
    K_ESCAPE,
    KEYDOWN,
    QUIT,
)


class KeyboardController(BaseController):
    def __init__(self, in_sim=True):
        BaseController.__init__(self, "Keyboard Controller", handle_quit=False)
        self.curr_waypoint = 0
        self.in_sim = in_sim


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
        return angle


    def select_action_from_state(self, env, state):

        if self.in_sim:
            env.set_waypoint(self.curr_waypoint)

        boat_x, boat_y, boat_speed, _, boat_angle, boat_ang_vel, ocean_current_x, ocean_current_y, obstacles = state

        waypoint = [env.waypoints[self.curr_waypoint].lon, env.waypoints[self.curr_waypoint].lat]
        dist = LatLon.dist(LatLon(boat_y, boat_x), LatLon(waypoint[1], waypoint[0]))

        if abs(dist) < 0.05:
            self.curr_waypoint = (self.curr_waypoint + 1) % len(env.waypoints)

        delta_x, delta_y = self.get_distances(waypoint, boat_x, boat_y)
        angle = self.get_required_angle_change(boat_angle, delta_x, delta_y)
        angle = angle % 360
        print(min(angle, angle - 360, key=abs))

        # Accelerating by specified values for one frame
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE or event.type == QUIT:
                    env.close()
                if event.key == K_UP:
                    return Action(0, 60)
                if event.key == K_DOWN:
                    return Action(0, -60)
                if event.key == K_LEFT:
                    return Action(1, 60*60)
                if event.key == K_RIGHT:
                    return Action(1, -60*60)
            if event.type == QUIT:
                env.close()

        return Action(0, 0)
