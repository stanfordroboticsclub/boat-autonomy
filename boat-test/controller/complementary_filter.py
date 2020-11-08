import math
import pygame
from time import sleep, time

from controller.base_controller import BaseController
from boat_simulation.simple import Action

from pygame.locals import (
    K_UP,
    K_DOWN,
    K_LEFT,
    K_RIGHT,
    K_ESCAPE,
    KEYDOWN,
    QUIT,
)


class ComplementaryFilterController(BaseController):
    def __init__(self):
        BaseController.__init__(self, "Complementary Filter Test", handle_quit=False)
        self.filt = ComplementaryFilter(0.99)
        self.lastGyro = 0
        self.lastMag = 0
        self.lastPub = 0

    def update_filter(self, state):
        if time() - self.lastMag > 0.10:
            heading = state[1]
            self.filt.update_mag(heading)
            self.lastMag = time()

        if time() - self.lastGyro > 0.01:
            self.filt.update_gyro(-state[0])
            self.lastGyro = time()

        if time() - self.lastPub > 0.05:
            angle = self.filt.get_angle()
            print(angle)
            self.lastPub = time()

    def get_user_input(self, env):
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE or event.type == QUIT:
                    env.close()
                if event.key == K_UP:
                    return Action(0, 40)
                if event.key == K_DOWN:
                    return Action(0, -40)
                if event.key == K_LEFT:
                    return Action(1, 60)
                if event.key == K_RIGHT:
                    return Action(1, -60)

            if event.type == QUIT:
                env.close()

        return Action(0, 0)

    def select_action_from_state(self, env, state):
        self.update_filter(state)
        # print(state)
        return self.get_user_input(env)


# yoinked from https://github.com/stanfordroboticsclub/RoverIMU/blob/master/compass.py
class ComplementaryFilter:
    def __init__(self, K):
        self.K = K
        self.lastSin = None
        self.lastCos = None
        self.lastGyroTime = None
        self.lastGyro = 0

    def update_mag(self,heading):
        # heading is in degrees
        rad = math.radians(heading)

        if self.lastSin == None or self.lastSin == None:
            self.lastSin = math.sin(rad)
            self.lastCos = math.cos(rad)
        else:
            self.lastSin = self.K * self.lastSin + (1-self.K) * math.sin(rad)
            self.lastCos = self.K * self.lastCos + (1-self.K) * math.cos(rad)

    def update_gyro(self,omega):
        # omega is in deg/s
        if self.lastGyroTime == None:
            self.lastGyroTime = time()
            return
        if self.lastSin == None or self.lastSin == None:
            return

        omega_rad = math.radians(omega)
        delta_t = time() - self.lastGyroTime
        self.lastGyroTime = time()

        rad = math.atan2(self.lastSin, self.lastCos) + omega_rad * delta_t
        self.lastSin = math.sin(rad)
        self.lastCos = math.cos(rad)

        self.lastGyro = (self.lastGyro + omega * delta_t) % 360

    def get_angle(self):
        rad = math.atan2(self.lastSin, self.lastCos)
        return math.degrees(rad) % 360
