import pygame

from pygame.locals import (
    K_ESCAPE,
    KEYDOWN,
    QUIT,
)


class BaseController(object):
    def __init__(self, name, handle_quit=True):
        super(BaseController, self).__init__()
        self.name = name

        # if handle_quit is True, the controller will first check for quit events. This parameter should
        # only be set to false for the keyboard controller.
        self.handle_quit = handle_quit

    # private method to avoid being called outside of subclass
    def select_action_from_state(self, env, state):
        # implemented by subclasses
        pass

    def choose_action(self, env, state):
        if self.handle_quit:
            for event in pygame.event.get():
                if event.type == KEYDOWN:
                    if event.key == K_ESCAPE or event.type == QUIT:
                        env.close()
                if event.type == QUIT:
                    env.close()

        return self.select_action_from_state(env, state)
