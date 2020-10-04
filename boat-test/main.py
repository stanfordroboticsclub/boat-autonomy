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


def check_for_quit():
    for event in pygame.event.get():
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE or event.type == QUIT:
                env.close()
        if event.type == QUIT:
            env.close()


def user_control(env):
    for event in pygame.event.get():
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE or event.type == QUIT:
                env.close()
            if event.key == K_UP:
                return Action(0, 5)
            if event.key == K_DOWN:
                return Action(0, -5)
            if event.key == K_LEFT:
                return Action(1, 5)
            if event.key == K_RIGHT:
                return Action(1, -5)
        if event.type == QUIT:
            env.close()

    return Action(0, 0)


def choose_action(env, state):
    # replace this code with autonomy code; output an Action given the state as input
    return Action(0, 0)


if __name__ == '__main__':
    env = SimpleBoatSim()
    state = env.reset()

    keyboard_control = True

    while True:
        action = Action(0, 0)

        if keyboard_control:
            action = user_control(env)
        else:
            check_for_quit()
            action = choose_action(env, state)

        state = env.step(action)[0]
        env.render()
