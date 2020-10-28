import pygame
import sys
import numpy as np

from boat_simulation.simulation_sprites import *

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

BOAT_WIDTH = 22
BOAT_HEIGHT = 44

VEL_SCALE = 1/60
ANGLE_SCALE = 1/60


class SimpleBoatSim(object):
    """boat simulation"""

    def __init__(self, max_obstacles=10, obs_chance=5e-2, current_level=1, state_mode="ground_truth"):
        super(SimpleBoatSim, self).__init__()
        pygame.init()
        self.screen = None
        self.boat_sprite = BoatSprite(BOAT_WIDTH, BOAT_HEIGHT)

        self.boat_coords = (SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2)
        # self.boat_coords = ((BOAT_WIDTH) / 2, SCREEN_HEIGHT - (BOAT_HEIGHT) / 2)

        self.waypoints = [self.boat_coords]

        self.speed = 0
        self.angular_speed = 0
        self.angle = 0

        self.real_speed = 0
        self.real_angular_speed = 0

        self.obstacles = pygame.sprite.Group()
        self.max_obstacles = max_obstacles
        self.obs_chance = obs_chance
        self.clock = pygame.time.Clock()

        self.total_time = 0
        self.current_level = current_level
        self.state_mode = state_mode

    def get_ground_truth_state(self):
        state = [self.boat_coords[0], self.boat_coords[1], self.real_speed, self.angle, self.real_angular_speed  / (1/60)]

        obs_states = []
        for obs in self.obstacles:
            obs_states.append([obs.radius, obs.rect.x, obs.rect.y, obs.velocity[0], obs.velocity[1]])
        state.append(obs_states)

        return state

    def get_noisy_state(self, noise_level=.5, prob_delete=.1):
        truth = self.get_ground_truth_state()

        for i in range(5):
            truth[i] += .5*(np.random.uniform() - .5)

        # Delete a few obstacles
        obs_states = truth[5]
        obs_states = [obs for obs in obs_states if np.random.uniform() < prob_delete]

        # Add noise to obstacle states
        for i in range(len(obs_states)):
            obs_states[i] = [k + .5*(np.random.uniform() - .5) for k in obs_states[i]]

        truth[5] = obs_states
        return truth

    def get_sensor_observation(self, d_theta, ang_vel_noise=.5, heading_noise=1):
        # [gyro angular velocity (deg/s), magnetometer heading (degrees)]
        return [ang_vel_noise*np.random.uniform(-1, 1) + d_theta/(1/60),
            heading_noise*np.random.uniform(-1, 1) + self.angle]

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
        elif action.type == 2:
            self.angular_speed += action.value[0]
            self.speed += action.value[1]

        intended_boat_dx = VEL_SCALE * self.speed * np.sin(np.pi * self.angle / 180)
        intended_boat_dy = VEL_SCALE * self.speed * np.cos(np.pi * self.angle / 180)

        # Account for ocean currents
        ocean_current_x, ocean_current_y = self.compute_ocean_current(self.boat_coords[0], self.boat_coords[1])

        boat_dx = intended_boat_dx - ocean_current_x
        boat_dy = intended_boat_dy - ocean_current_y

        self.real_speed = np.sqrt(boat_dx**2 + boat_dy**2) / VEL_SCALE

        projection = (intended_boat_dx * boat_dx + intended_boat_dy * boat_dy) / VEL_SCALE * self.speed
        if projection < 0:
            self.real_speed *= -1

        self.boat_coords = (self.boat_coords[0] - boat_dx,
                            self.boat_coords[1] - boat_dy)

        # Currents apply a torque on the boat and make it rotate
        d_theta = ANGLE_SCALE * self.angular_speed
        d_theta += self.current_rotation()
        # print(d_theta)

        self.angle += d_theta
        self.real_angular_speed = d_theta

        if np.random.uniform() < self.obs_chance and len(self.obstacles) < self.max_obstacles:
            while True:
                proposed_obstacle = ObstacleSprite(np.random.randint(10, 20),
                                                   coords=(np.random.uniform(0, SCREEN_WIDTH),
                                                           np.random.uniform(0, SCREEN_HEIGHT)),
                                                   live_counter=np.random.randint(500, 1e3))
                if not pygame.sprite.collide_rect(proposed_obstacle, self.boat_sprite):
                    break
            self.obstacles.add(proposed_obstacle)

        # get the new state of the environment
        if self.state_mode == "ground_truth":
            state = self.get_ground_truth_state()
        elif self.state_mode == "noisy":
            state = self.get_noisy_state()
        elif self.state_mode == "sensor":
            state = self.get_sensor_observation(d_theta)

        # gets all sprites in the obstacles Group that have collided with the boat
        collision = pygame.sprite.spritecollide(self.boat_sprite, self.obstacles, True)

        self.total_time += 1/60

        end_sim = False
        if len(collision) > 0:
            end_sim = True

        return state, 0, end_sim, None

    def proj(self, a, b):   # Project a onto b
        coefficient = (a[0] * b[0] + a[1] * b[1]) / (b[0]**2 + b[1]**2)
        return (coefficient * b[0], coefficient * b[1])

    def current_rotation(self):
        top_pos = (self.boat_coords[0] - BOAT_HEIGHT/2 * np.sin(np.pi * self.angle / 180),
            self.boat_coords[1] - BOAT_HEIGHT/2 * np.cos(np.pi * self.angle / 180))
        bottom_pos = (self.boat_coords[0] + BOAT_HEIGHT/2 * np.sin(np.pi * self.angle / 180),
            self.boat_coords[1] + BOAT_HEIGHT/2 * np.cos(np.pi * self.angle / 180))

        top_x, top_y = self.compute_ocean_current(top_pos[0], top_pos[1])
        bot_x, bot_y = self.compute_ocean_current(bottom_pos[0], bottom_pos[1])

        # project <top_x, top_y> onto the 'boat vector'
        boat_vector = (top_pos[0] - bottom_pos[0], top_pos[1] - bottom_pos[1])
        top_along = self.proj((top_x, top_y), boat_vector)
        bot_along = self.proj((bot_x, bot_y), boat_vector)

        top_perp = (top_x - top_along[0], top_y - top_along[1])
        bot_perp = (bot_x - bot_along[0], bot_y - bot_along[1])

        total = ((top_perp[0] + bot_perp[0])/2, (top_perp[1] + bot_perp[1])/2)

        return np.sqrt(total[0]**2 + total[1]**2)

    def reset(self):
        """Resets simulation and returns initial state"""
        self.boat_coords = (SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2)
        # self.boat_coords = ((BOAT_WIDTH) / 2, SCREEN_HEIGHT - (BOAT_HEIGHT) / 2)
        self.angle = 0

        self.total_time = 0

        self.speed = 0
        self.angular_speed = 0

        self.real_speed = 0
        self.real_angular_speed = 0

        obs_list = self.obstacles.sprites()
        for obs in obs_list:
            obs.kill()

        self.obstacles = pygame.sprite.Group()
        self.waypoints = self.generate_data(15, (BOAT_WIDTH) / 2, SCREEN_WIDTH - (BOAT_WIDTH) / 2, BOAT_HEIGHT / 2,
                                            SCREEN_HEIGHT - (BOAT_HEIGHT) / 2)
        self.waypoints.append(self.boat_coords)

        self.waypoints = self.compute_convex_hull(self.waypoints)

        # Ocean currents
        # Currently follows sin(ax+by), cos(cx+dy)

        ocean_current_multiplier = 1e-2
        self.ocean_current_a = np.random.uniform() * ocean_current_multiplier
        self.ocean_current_b = np.random.uniform() * ocean_current_multiplier
        self.ocean_current_c = np.random.uniform() * ocean_current_multiplier
        self.ocean_current_d = np.random.uniform() * ocean_current_multiplier
        self.ocean_current_e = np.random.uniform() * ocean_current_multiplier

        if self.state_mode == "ground_truth":
            state = self.get_ground_truth_state()
        elif self.state_mode == "noisy":
            state = self.get_noisy_state()
        elif self.state_mode == "sensor":
            state = self.get_sensor_observation(0)

        return state

    def render(self):
        """Repeatedly call this function in your loop if you want to visualize the simulation"""

        if self.screen is None:
            self.screen = pygame.display.set_mode([SCREEN_WIDTH, SCREEN_HEIGHT])

        self.screen.fill((3, 169, 252))
        pygame.draw.lines(self.screen, (255, 221, 128), True, self.waypoints, 10)

        if self.current_level > 0:
            self.render_ocean_currents()
        self.render_boat()
        self.render_obstacles()

        # draw first waypoint

        pygame.draw.circle(self.screen, (255, 0, 0), self.waypoints[0], 10)

        # print current boat velocity
        font = pygame.font.SysFont(None, 24)

        vel_text = font.render(f"vel: %s" % (VEL_SCALE * self.speed), True, (255, 255, 255))
        ang_vel_text = font.render(f"ang. vel applied: %s" % (ANGLE_SCALE * self.angular_speed / (1/60)), True, (255, 255, 255))
        ang_text = font.render(f"ang: %s" % (self.angle), True, (255, 255, 255))

        self.screen.blit(vel_text, (20, 20))
        self.screen.blit(ang_vel_text, (20, 50))
        self.screen.blit(ang_text, (20, 80))

        pygame.display.update()

        # cap the framerate at 60 fps
        self.clock.tick(60)
        # print(self.clock.get_fps())

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

    def render_ocean_currents(self):
        for x in range(0, SCREEN_WIDTH + 1, 100):
            for y in range(0, SCREEN_HEIGHT + 1, 100):
                ocean_x, ocean_y = self.compute_ocean_current(x, y)
                ocean_x *= 300 / self.current_level
                ocean_y *= 300 / self.current_level
                pygame.draw.line(self.screen, (10, 50, 255), (x, y), (x + ocean_x, y + ocean_y), 3)
                pygame.draw.circle(self.screen, (10, 50, 255), (x + ocean_x, y + ocean_y), 5)

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

    def compute_ocean_current(self, x, y):
        ocean_current_x = self.current_level * 0.1 * np.cos(
            self.ocean_current_a * x + self.ocean_current_b * y + 15 * self.current_level * self.ocean_current_e * self.total_time)
        ocean_current_y = self.current_level * 0.1 * np.cos(
            self.ocean_current_c * x + self.ocean_current_d * y + 15 * self.current_level * self.ocean_current_e * self.total_time)

        return ocean_current_x, ocean_current_y


class Action(object):
    """class representing action boat is taking."""

    def __init__(self, type, value):
        super(Action, self).__init__()
        self.type = type  # 0 is forward/back, 1 is turn
        self.value = value  # How much to change (angular) velocity
