import pygame
import sys
import numpy as np

from boat_simulation.simulation_sprites import *

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

BOAT_WIDTH = 22
BOAT_HEIGHT = 44

VEL_SCALE = .01
ANGLE_SCALE = .01


class SimpleBoatSim(object):
    """boat simulation"""

    def __init__(self, max_obstacles=10, obs_chance=5e-3):
        super(SimpleBoatSim, self).__init__()
        pygame.init()
        self.screen = None
        self.boat_sprite = BoatSprite(BOAT_WIDTH, BOAT_HEIGHT)
        self.boat_coords = ((BOAT_WIDTH)/2, SCREEN_HEIGHT - (BOAT_HEIGHT)/2)
        self.waypoints = [self.boat_coords]

        self.speed = 0
        self.angular_speed = 0
        self.angle = 0

        self.obstacles = pygame.sprite.Group()
        self.max_obstacles = max_obstacles
        self.obs_chance = obs_chance

    def step(self, action):
        """
        Takes one step forward of the simulation given an action.
        Will return the new state (observation robot receives), a reward,
        if the simulation has terminated, and any additional info.

        State: [boat x, boat y, boat speed, boat angle, boat angular velocity,
        [[obstacle 1 radius, obstacte 1 x, obstacle 1 y, obstacle 1 x velocity,
        obstacle 1 y velocity], [obstacle 2 radius, ...], ...]]
        """
        if action.type == 0:
            self.speed = self.speed + action.value
        elif action.type == 1:
            self.angular_speed += action.value

        self.boat_coords = (self.boat_coords[0] - VEL_SCALE*self.speed*np.sin(np.pi*self.angle/180),
                            self.boat_coords[1] - VEL_SCALE*self.speed*np.cos(np.pi*self.angle/180))
        self.angle += ANGLE_SCALE*self.angular_speed

        state = [self.boat_coords[0], self.boat_coords[1], self.speed, self.angle, self.angular_speed]

        if np.random.uniform() < self.obs_chance and len(self.obstacles) < self.max_obstacles:
            while True:
                proposed_obstacle = ObstacleSprite(np.random.randint(10, 20),
                    coords=(np.random.uniform(0, SCREEN_WIDTH), np.random.uniform(0, SCREEN_HEIGHT)),
                    live_counter=np.random.randint(4e3, 5e3))
                if not pygame.sprite.collide_rect(proposed_obstacle, self.boat_sprite):
                    break
            self.obstacles.add(proposed_obstacle)

        obs_states = []
        for obs in self.obstacles:
            obs_states.append([obs.radius, obs.rect.x, obs.rect.y, obs.velocity[0], obs.velocity[1]])
        state.append(obs_states)

        return state, 0, False, None

    def reset(self):
        """Resets simulation and returns initial state"""
        self.boat_coords = ((BOAT_WIDTH)/2, SCREEN_HEIGHT - (BOAT_HEIGHT)/2)
        self.angle = 0

        self.speed = 0
        self.angular_speed = 0

        obs_list = self.obstacles.sprites()
        for obs in obs_list:
            obs.kill()

        self.obstacles = pygame.sprite.Group()
        state = [self.boat_coords[0], self.boat_coords[1], self.speed, self.angle, self.angular_speed, []]

        self.waypoints = self.generate_data(15, (BOAT_WIDTH)/2, SCREEN_WIDTH - (BOAT_WIDTH)/2, BOAT_HEIGHT/2, SCREEN_HEIGHT - (BOAT_HEIGHT)/2)
        self.waypoints.append(self.boat_coords)

        self.waypoints = self.compute_convex_hull(self.waypoints)

        return state

    def render(self):
        """Repeatedly call this function in your loop if you want to visualize the simulation"""

        if self.screen is None:
            self.screen = pygame.display.set_mode([SCREEN_WIDTH, SCREEN_HEIGHT])

        self.screen.fill((3, 169, 252))
        pygame.draw.lines(self.screen, (255, 221, 128), True, self.waypoints, 10)
        self.render_boat()
        self.render_obstacles()
        pygame.display.update()

    def render_boat(self):
        self.boat_sprite.rotated_surf = pygame.transform.rotate(self.boat_sprite.surf, self.angle)
        self.boat_sprite.rect = self.boat_sprite.rotated_surf.get_rect(center=self.boat_coords)
        self.screen.blit(self.boat_sprite.rotated_surf, (self.boat_sprite.rect.x, self.boat_sprite.rect.y))
        self.boat_sprite.step()

    def render_obstacles(self):
        obs_list = self.obstacles.sprites()
        for obs in obs_list:
            if obs.step() == -1:
                obs.kill()

        for obs in self.obstacles:
            obs.curr_coords = (obs.curr_coords[0] + obs.velocity[0], obs.curr_coords[1] + obs.velocity[1])
            obs.rect = obs.surf.get_rect(center=(obs.curr_coords[0], obs.curr_coords[1]))
            self.screen.blit(obs.surf, (obs.rect.x, obs.rect.y))

    def close(self):
        pygame.quit()
        sys.exit(0)

    def generate_data(self, n, low_x, high_x, low_y, high_y):
        points = []

        for i in range(n):
            points.append((np.random.uniform(low_x, high_x), np.random.uniform(low_y, high_y)))

        return points

    def compute_min_x(self, data):
        min_idx = 0

        for i in range(len(data)):
            if (data[i][0] < data[min_idx][0] or
                (data[i][0] == data[min_idx][0] and data[i][1] < data[min_idx][1])):

                min_idx = i

        return min_idx

    def compute_convex_hull(self, data):
        p0_idx = self.compute_min_x(data)
        p0 = data[p0_idx]
        hull = [p0]
        data.pop(p0_idx)

        data.sort(key=lambda x: (x[1] - p0[1]) / (x[0] - p0[0]))

        hull.append(data.pop(0))
        hull.append(data.pop(0))

        while len(data) > 0:
            latest = data.pop(0)

            while True:
                v = (latest[0] - hull[-1][0], latest[1] - hull[-1][1])
                u = (hull[-2][0] - hull[-1][0], hull[-2][1] - hull[-1][1])

                cross = v[0] * u[1] - v[1] * u[0]

                if cross >= 0:
                    break

                hull.pop(-1)
            hull.append(latest)

        return hull


class Action(object):
    """class representing action boat is taking."""

    def __init__(self, type, value):
        super(Action, self).__init__()
        self.type = type        # 0 is forward/back, 1 is turn
        self.value = value      # How much to change (angular) velocity
