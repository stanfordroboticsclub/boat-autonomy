import pygame
from boat_simulation.simple import SimpleBoatSim, Action

from pygame.locals import (
    K_UP,
    K_DOWN,
    K_LEFT,
    K_RIGHT,
    K_ESCAPE,
    KEYDOWN,
    QUIT,
)


if __name__ == '__main__':
    env = SimpleBoatSim()
    state = env.reset()

    while True:
        action = Action(0, 0)

        # Replace this for loop with autonomy code; output an Action given the state as input.
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE or event.type == QUIT:
                    env.close()
                if event.key == K_UP:
                    action = Action(0, 5)
                if event.key == K_DOWN:
                    action = Action(0, -5)
                if event.key == K_LEFT:
                    action = Action(1, 5)
                if event.key == K_RIGHT:
                    action = Action(1, -5)
            if event.type == QUIT:
                env.close()

        state = env.step(action)[0]
        env.render()
