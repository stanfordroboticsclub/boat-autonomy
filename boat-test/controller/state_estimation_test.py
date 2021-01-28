import pygame
from controller.base_controller import BaseController
from boat_simulation.simple import Action
from boat_simulation.latlon import LatLon
import numpy as np

from state_estimators.complementary_filter import ComplementaryFilter

from pygame.locals import (
    K_UP,
    K_DOWN,
    K_LEFT,
    K_RIGHT,
    K_ESCAPE,
    KEYDOWN,
    QUIT,
)


class SETestController(BaseController):
    def __init__(self, in_sim=True):
        BaseController.__init__(self, "SETest", handle_quit=False)
        self.curr_waypoint = 0
        self.in_sim = in_sim
        self.complementary_filter = ComplementaryFilter(new_reading_weight=0.9)


    def select_action_from_state(self, env, state):
        if self.in_sim:
            env.set_waypoint(self.curr_waypoint)

        # boat_x, boat_y, boat_speed, _, boat_angle, boat_ang_vel, ocean_current_x, ocean_current_y, obstacles = state
        # print(state)
        omega, theta_m = state

        self.complementary_filter.update_magnetometer(theta_m)
        self.complementary_filter.update_gyro(omega)

        print(self.complementary_filter.get_heading())

        # waypoint = [env.waypoints[self.curr_waypoint].lon, env.waypoints[self.curr_waypoint].lat]
        # dist = LatLon.dist(LatLon(boat_y, boat_x), LatLon(waypoint[1], waypoint[0]))
        #
        # if abs(dist) < 0.05:
        #     self.curr_waypoint = (self.curr_waypoint + 1) % len(env.waypoints)

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
