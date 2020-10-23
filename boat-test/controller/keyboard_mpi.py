import pygame
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


class KeyboardMPI(BaseController):
    def __init__(self):
        BaseController.__init__(self, "MPI Keyboard Controller", handle_quit=False)

    def select_action_from_state(self, events, state):
        for event in events:
            if event == K_UP:
                return Action(0, 40)
            if event == K_DOWN:
                return Action(0, -40)
            if event == K_LEFT:
                return Action(1, 60)
            if event == K_RIGHT:
                return Action(1, -60)

        return Action(0, 0)
