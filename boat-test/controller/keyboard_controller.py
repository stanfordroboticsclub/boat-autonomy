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


class KeyboardController(BaseController):
    def __init__(self):
        BaseController.__init__(self, "Keyboard Controller", handle_quit=False)

    def select_action_from_state(self, env, state):
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
