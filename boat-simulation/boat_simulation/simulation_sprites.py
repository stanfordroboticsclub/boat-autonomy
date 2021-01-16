import pygame
import numpy as np


class BoatSprite(pygame.sprite.Sprite):
    """wanna sprite boat"""

    def __init__(self, boat_width, boat_height):
        super(BoatSprite, self).__init__()
        self.surf = pygame.Surface((boat_width, boat_height), pygame.SRCALPHA)
        self.surf.fill((204, 71, 0))
        pygame.draw.circle(self.surf, (0, 0, 255), (boat_width / 2, 10), 5)

        self.rotated_surf = self.surf
        self.rect = self.surf.get_rect()

    def step(self):
        return 0


class ObstacleSprite(pygame.sprite.Sprite):
    """wanna sprite obstacle"""

    def __init__(self, radius, coords=(0, 0), live_counter=5e3, velocity=None):
        super(ObstacleSprite, self).__init__()
        self.surf = pygame.Surface((2 * radius, 2 * radius), pygame.SRCALPHA)
        pygame.draw.circle(self.surf, (0, 125, 31), (radius, radius), radius)
        self.rect = self.surf.get_rect(center=coords)

        self.curr_coords = coords
        self.radius = radius

        if velocity is None:
            velocity = [np.random.uniform(-1, 1), np.random.uniform(-1, 1)]
            # velocity = [0, 0]
        self.velocity = velocity
        self.live_counter = live_counter

    def step(self):
        self.live_counter -= 1
        if self.live_counter == 0:
            return -1
        return 0
