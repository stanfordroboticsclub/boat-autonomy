import pygame
import sys
import numpy as np
from boat_simulation.latlon import LatLon
from boat_simulation.simulation_sprites import *

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

SCREEN_WIDTH_M = 20  # meters
SCREEN_HEIGHT_M = (SCREEN_WIDTH_M / SCREEN_WIDTH) * SCREEN_HEIGHT  # meters

# change in latitude is change in y, change in longitude is change in x
TOP_LEFT_LATLON = LatLon(7.399640, 134.457619)
BOT_RIGHT_LATLON = TOP_LEFT_LATLON.add_dist(SCREEN_WIDTH_M, SCREEN_HEIGHT_M)

PIXELS_PER_METER = SCREEN_WIDTH / SCREEN_WIDTH_M

BOAT_WIDTH = 0.5    # meters
BOAT_HEIGHT = 1     # meters
BOAT_MASS = 5       # kg

VEL_SCALE = 1 / 60
ANGLE_SCALE = 1 / 60


def latlon_to_xy(pos):
    left_point = LatLon(pos.lat, TOP_LEFT_LATLON.lon)
    top_point = LatLon(TOP_LEFT_LATLON.lat, pos.lon)

    x = PIXELS_PER_METER * LatLon.dist(pos, left_point)
    y = PIXELS_PER_METER * LatLon.dist(pos, top_point)

    # LatLon.dist always returns positive values, so based on whether pos is
    # above or below the top left, change sign

    if pos.lat < TOP_LEFT_LATLON.lat:
        y *= -1
    if pos.lon < TOP_LEFT_LATLON.lon:
        x *= -1

    return (x, y)


def xy_to_latlon(x, y):
    loc = TOP_LEFT_LATLON.add_dist(x / PIXELS_PER_METER,
                                   y / PIXELS_PER_METER)
    return loc


class SimpleBoatSim(object):
    """boat simulation"""

    def __init__(self, max_obstacles=10, obs_chance=5e-2, current_level=1, state_mode="ground_truth", apply_drag_forces=True):
        super(SimpleBoatSim, self).__init__()

        print(f"TOP_LEFT_LATLON: {TOP_LEFT_LATLON}")
        print(f"BOT_RIGHT_LATLON: {BOT_RIGHT_LATLON}")

        pygame.init()
        self.screen = None
        self.boat_sprite = BoatSprite(PIXELS_PER_METER * BOAT_WIDTH, PIXELS_PER_METER * BOAT_HEIGHT)

        self.boat_coords = TOP_LEFT_LATLON.add_dist(SCREEN_WIDTH_M / 2, SCREEN_HEIGHT_M / 2)  # lat lon coordinates
        print(f"INITIAL BOAT COORDS: {self.boat_coords}")
        # self.boat_coords = ((BOAT_WIDTH) / 2, SCREEN_HEIGHT - (BOAT_HEIGHT) / 2)

        self.waypoints = []
        self.waypoints_xy = []

        self.curr_waypoint = -1

        self.speed = 0  # m/s
        self.delta_speed_remaining = 0
        self.angular_speed = 0  # deg/sec
        self.delta_angular_speed_remaining = 0
        self.angle = 0  # deg

        self.real_speed = 0  # m/s
        self.real_angular_speed = 0  # deg/sec

        self.obstacles = pygame.sprite.Group()
        self.max_obstacles = max_obstacles
        self.obs_chance = obs_chance
        self.clock = pygame.time.Clock()

        self.total_time = 0
        self.current_level = current_level
        self.state_mode = state_mode

        self.path_to_plot = None
        self.apply_drag_forces = apply_drag_forces

        self.voronoi_graph = None
        self.voronoi_path = None

    def get_ground_truth_state(self):
        state = [self.boat_coords.lon, self.boat_coords.lat, self.real_speed, self.angle, self.real_angular_speed]

        obs_states = []
        for obs in self.obstacles:
            obs_latlon = xy_to_latlon(obs.rect.x, obs.rect.y)
            obs_states.append(
                [obs.radius * SCREEN_WIDTH_M / SCREEN_WIDTH, obs_latlon.lon, obs_latlon.lat, obs.velocity[0],
                 obs.velocity[1]])
        state.append(obs_states)
        # state.append([self.waypoints[self.curr_waypoint].lat, self.waypoints[self.curr_waypoint].lon])

        return state

    def get_noisy_state(self, noise_level=.5, prob_delete=.1):
        truth = self.get_ground_truth_state()

        for i in range(5):
            truth[i] += .5 * (np.random.uniform() - .5)

        # Delete a few obstacles
        obs_states = truth[5]
        obs_states = [obs for obs in obs_states if np.random.uniform() < prob_delete]

        # Add noise to obstacle states
        for i in range(len(obs_states)):
            obs_states[i] = [k + .5 * (np.random.uniform() - .5) for k in obs_states[i]]

        truth[5] = obs_states
        return truth

    def get_sensor_observation(self, ang_vel_noise=.5, heading_noise=1):
        # [gyro angular velocity (deg/s), magnetometer heading (degrees)]
        truth = self.get_ground_truth_state()

        orientation_sensors = [ang_vel_noise * np.random.uniform(-1, 1) + self.real_angular_speed,
                heading_noise * np.random.uniform(-1, 1) + self.angle]

        state = truth[0:3]
        state += orientation_sensors
        # state.append(orientation_sensors[0])
        state += truth[5:]

        return state

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
            # self.speed is in meters/sec. Input action value is accel in m/s^2
            self.delta_speed_remaining += (VEL_SCALE * action.value)
        elif action.type == 1:
            # self.angular_speed is in degrees/sec. Input action value is angular accel in degrees/s^2
            self.delta_angular_speed_remaining += (ANGLE_SCALE * action.value)
        elif action.type == 2:
            self.delta_angular_speed_remaining += ANGLE_SCALE * action.value[0]
            self.delta_speed_remaining += VEL_SCALE * action.value[1]

        if self.apply_drag_forces:
            speed_step = 0.1
            angular_speed_step = 3
        else:
            speed_step = abs(self.delta_speed_remaining)
            angular_speed_step = abs(self.delta_angular_speed_remaining)

        if self.delta_speed_remaining > 0:
            self.speed += speed_step
            self.delta_speed_remaining -= speed_step
        elif self.delta_speed_remaining < 0:
            self.speed -= speed_step
            self.delta_speed_remaining += speed_step

        if self.delta_angular_speed_remaining > 0:
            self.angular_speed += angular_speed_step
            self.delta_angular_speed_remaining -= angular_speed_step
        elif self.delta_angular_speed_remaining < 0:
            self.angular_speed -= angular_speed_step
            self.delta_angular_speed_remaining += angular_speed_step

        # speed is in meters/sec
        intended_boat_dx = VEL_SCALE * self.speed * np.sin(np.deg2rad(self.angle))  # meters/frame
        intended_boat_dy = VEL_SCALE * self.speed * np.cos(np.deg2rad(self.angle))  # meters/frame

        # Account for ocean currents
        ocean_current_x, ocean_current_y = self.compute_ocean_current(self.boat_coords)  # meters/sec

        # make currents meters/frame
        ocean_current_x *= VEL_SCALE
        ocean_current_y *= VEL_SCALE

        # print(f"current magnitude in cm/sec: {100 * np.sqrt((ocean_current_x / VEL_SCALE)**2 + (ocean_current_y / VEL_SCALE)**2)}")

        boat_dx = intended_boat_dx - ocean_current_x  # meters/frame
        boat_dy = intended_boat_dy - ocean_current_y  # meters/frame

        if self.apply_drag_forces:
            self.apply_drag()

        self.real_speed = np.sqrt(boat_dx ** 2 + boat_dy ** 2) / VEL_SCALE  # meters/sec

        if self.speed != 0:
            projection = (intended_boat_dx * boat_dx + intended_boat_dy * boat_dy) / (VEL_SCALE * self.speed)
            if projection < 0:
                self.real_speed *= -1

        self.boat_coords = self.boat_coords.add_dist(-boat_dx, -boat_dy)
        # print(self.boat_coords)

        # Currents apply a torque on the boat and make it rotate
        d_theta = ANGLE_SCALE * self.angular_speed  # deg/frame
        d_theta += self.current_rotation()

        self.angle += d_theta
        self.real_angular_speed = d_theta / ANGLE_SCALE

        if np.random.uniform() < self.obs_chance and len(self.obstacles) < self.max_obstacles:
            while True:
                ox = np.random.uniform(0, SCREEN_WIDTH)
                oy = np.random.uniform(0, SCREEN_HEIGHT)
                r = np.random.randint(10, 20)
                proposed_obstacle = ObstacleSprite(radius=r,
                                                   coords=(ox, oy),
                                                   live_counter=np.random.randint(500, 1e3))
                if self.waypoint_is_valid(ox, oy, r):
                    break
                # if not pygame.sprite.collide_rect(proposed_obstacle, self.boat_sprite) and self.waypoint_is_valid(ox, oy, r):
                #     break
            self.obstacles.add(proposed_obstacle)

        # get the new state of the environment
        state = None
        if self.state_mode == "ground_truth":
            state = self.get_ground_truth_state()
        elif self.state_mode == "noisy":
            state = self.get_noisy_state()
        elif self.state_mode == "sensor":
            state = self.get_sensor_observation(d_theta)

        # gets all sprites in the obstacles Group that have collided with the boat
        collision = pygame.sprite.spritecollide(self.boat_sprite, self.obstacles, True)

        self.total_time += 1 / 60

        end_sim = False
        if len(collision) > 0:
            end_sim = True

        return state, 0, end_sim, None

    def plot_path(self, path):
        self.path_to_plot = path

    def waypoint_is_valid(self, x, y, r):
        obs_latlon = xy_to_latlon(x, y)
        if LatLon.dist(self.boat_coords, obs_latlon) < 2:
            return False

        for w in self.waypoints:
            if LatLon.dist(w, obs_latlon) < r*(SCREEN_WIDTH_M / SCREEN_WIDTH) + 2:
                return False

        return True

    # a and b are vectors in meters
    def proj(self, a, b):  # Project a onto b
        coefficient = (a[0] * b[0] + a[1] * b[1]) / (b[0] ** 2 + b[1] ** 2)
        return (coefficient * b[0], coefficient * b[1])

    def current_rotation(self):
        # top_pos = (self.boat_coords[0] - BOAT_HEIGHT/2 * np.sin(np.pi * self.angle / 180),
        #     self.boat_coords[1] - BOAT_HEIGHT/2 * np.cos(np.pi * self.angle / 180))

        # LatLon
        top_pos = self.boat_coords.add_dist(-BOAT_HEIGHT / 2 * np.sin(np.deg2rad(self.angle)),
                                            -BOAT_HEIGHT / 2 * np.cos(np.deg2rad(self.angle)))

        # bottom_pos = (self.boat_coords[0] + BOAT_HEIGHT/2 * np.sin(np.pi * self.angle / 180),
        #     self.boat_coords[1] + BOAT_HEIGHT/2 * np.cos(np.pi * self.angle / 180))

        # LatLon
        bottom_pos = self.boat_coords.add_dist(BOAT_HEIGHT / 2 * np.sin(np.deg2rad(self.angle)),
                                               BOAT_HEIGHT / 2 * np.cos(np.deg2rad(self.angle)))

        # current vector components in meters
        top_x, top_y = self.compute_ocean_current(top_pos)
        bot_x, bot_y = self.compute_ocean_current(bottom_pos)

        # project <top_x, top_y> onto the 'boat vector'
        boat_vector = (BOAT_HEIGHT * -np.sin(np.deg2rad(self.angle)), BOAT_HEIGHT * -np.cos(np.deg2rad(self.angle)))
        top_along = self.proj((top_x, top_y), boat_vector)
        bot_along = self.proj((bot_x, bot_y), boat_vector)

        # vectors in meters
        top_perp = (top_x - top_along[0], top_y - top_along[1])
        bot_perp = (bot_x - bot_along[0], bot_y - bot_along[1])

        # vector in meters
        total = ((top_perp[0] + bot_perp[0]) / 2, (top_perp[1] + bot_perp[1]) / 2)

        # cross product between total and -boat_vector

        cross_prod = -(total[0] * boat_vector[1] - total[1] * boat_vector[0])

        coeff = 1
        if cross_prod < 0:
            coeff = -1

        return coeff * np.sqrt(total[0] ** 2 + total[1] ** 2)

    def create_waypoints(self):
        self.waypoints_xy = self.generate_data(15, (BOAT_WIDTH) / 2, SCREEN_WIDTH - (BOAT_WIDTH) / 2, BOAT_HEIGHT / 2,
                                               SCREEN_HEIGHT - (BOAT_HEIGHT) / 2)
        self.waypoints_xy = self.compute_convex_hull(self.waypoints_xy)

        self.waypoints = []
        for w in self.waypoints_xy:
            self.waypoints.append(xy_to_latlon(w[0], w[1]))

    def set_waypoints(self, waypoints):
        self.waypoints = waypoints
        self.waypoints_xy = [latlon_to_xy(w) for w in self.waypoints]

    def reset(self):
        """Resets simulation and returns initial state"""
        self.boat_coords = TOP_LEFT_LATLON.add_dist(SCREEN_WIDTH_M / 2, SCREEN_HEIGHT_M / 2)  # lat lon coordinates
        # self.boat_coords = ((BOAT_WIDTH) / 2, SCREEN_HEIGHT - (BOAT_HEIGHT) / 2)
        self.angle = 0

        self.total_time = 0

        self.delta_speed_remaining = 0
        self.speed = 0
        self.delta_angular_speed_remaining = 0
        self.angular_speed = 0

        self.real_speed = 0
        self.real_angular_speed = 0

        obs_list = self.obstacles.sprites()
        for obs in obs_list:
            obs.kill()

        self.obstacles = pygame.sprite.Group()

        self.create_waypoints()

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

        pygame.event.get()

        if self.screen is None:
            self.screen = pygame.display.set_mode([SCREEN_WIDTH, SCREEN_HEIGHT])

        self.screen.fill((3, 169, 252))
        pygame.draw.lines(self.screen, (255, 221, 128), True, self.waypoints_xy, 10)

        if self.current_level > 0:
            self.render_ocean_currents()
        self.render_boat()
        self.render_obstacles()
        self.render_voronoi()

        # draw waypoint
        if self.curr_waypoint != -1:
            pygame.draw.circle(self.screen, (207, 106, 72), self.waypoints_xy[self.curr_waypoint], 10)

        if self.path_to_plot is not None:
            path_xy = [latlon_to_xy(k) for k in self.path_to_plot]
            pygame.draw.lines(self.screen, (137, 52, 235), False, path_xy, 5)

        # print current boat velocity
        font = pygame.font.SysFont(None, 24)

        vel_text = font.render(f"vel: %s m/sec" % round(self.speed, 5), True, (255, 255, 255))
        ang_vel_text = font.render(f"ang. vel applied: %s deg/sec" % round(self.angular_speed, 5), True,
                                   (255, 255, 255))
        ang_text = font.render(f"ang: %s deg" % round(self.angle, 5), True, (255, 255, 255))

        self.screen.blit(vel_text, (20, 20))
        self.screen.blit(ang_vel_text, (20, 50))
        self.screen.blit(ang_text, (20, 80))

        pygame.display.update()

        # cap the framerate at 60 fps
        self.clock.tick(60)
        # print(self.clock.get_fps())

    def set_waypoint(self, new_waypoint):
        self.curr_waypoint = new_waypoint

    def render_boat(self):
        self.boat_sprite.rotated_surf = pygame.transform.rotate(self.boat_sprite.surf, self.angle)
        boat_center_xy = latlon_to_xy(self.boat_coords)
        self.boat_sprite.rect = self.boat_sprite.rotated_surf.get_rect(center=boat_center_xy)
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
                # m/sec
                ocean_x, ocean_y = self.compute_ocean_current(
                    TOP_LEFT_LATLON.add_dist((x / SCREEN_WIDTH) * SCREEN_WIDTH_M,
                                             (y / SCREEN_HEIGHT) * SCREEN_HEIGHT_M))

                ocean_x = 100 * 45 * ocean_x / self.current_level
                ocean_y = 100 * 45 * ocean_y / self.current_level

                pygame.draw.line(self.screen, (10, 50, 255), (x, y), (x + ocean_x, y + ocean_y), 3)
                pygame.draw.circle(self.screen, (10, 50, 255), (x + ocean_x, y + ocean_y), 5)

    def render_voronoi(self):
        if self.voronoi_graph is not None:
            for i in range(len(self.voronoi_graph.points)):
                for j in self.voronoi_graph.edges[i]:
                    start = self.voronoi_graph.points[i]
                    end = self.voronoi_graph.points[j]
                    color = (137, 52, 235)

                    if i >= len(self.voronoi_graph.points) - 2 or j >= len(self.voronoi_graph.points) - 2:
                        color = (137, 235, 52)
                    pygame.draw.line(self.screen, color, start, end, 5)

            if self.voronoi_path is not None:
                path_points = []
                for i in self.voronoi_path:
                    path_points.append(self.voronoi_graph.points[i])

                color = (255, 255, 255)
                pygame.draw.lines(self.screen, color, False, path_points, 5)

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

    # returns ocean current components in m/sec. self.current_level is in cm/sec
    def compute_ocean_current(self, pos):
        x, y = latlon_to_xy(pos)

        pixels_per_fr_curr_level = PIXELS_PER_METER * self.current_level * VEL_SCALE / 100

        multiplier = pixels_per_fr_curr_level / np.sqrt(2)

        # 25 and 15 are magic constants right now
        ocean_current_x = multiplier * np.cos(
            self.ocean_current_a * x + self.ocean_current_b * y + 15 * (
                    self.current_level / 10) * self.ocean_current_e * self.total_time)
        ocean_current_y = multiplier * np.cos(
            self.ocean_current_c * x + self.ocean_current_d * y + 15 * (
                    self.current_level / 10) * self.ocean_current_e * self.total_time)

        ocean_current_x /= (PIXELS_PER_METER * VEL_SCALE)
        ocean_current_y /= (PIXELS_PER_METER * VEL_SCALE)

        return ocean_current_x, ocean_current_y

    def apply_drag(self):
        drag_coefficient = 1
        seawater_density = 1026  # kg/m^3
        depth_of_boat_under_water = 0.1  # meters, represents how much of the boat is underwater
        cross_section_area = BOAT_WIDTH * depth_of_boat_under_water

        drag_force = 0.5 * seawater_density * (self.speed ** 2) * drag_coefficient * cross_section_area  # Newtons

        drag_accel = drag_force / BOAT_MASS  # m/s^2

        # print(f"DRAG: {boat_dx}, {boat_dy}, speed={self.speed} --> {drag_accel}")

        drag_accel *= 0.05

        if self.speed > 0:
            self.speed -= drag_accel * VEL_SCALE
        else:
            self.speed += drag_accel * VEL_SCALE


class Action(object):
    """class representing action boat is taking."""

    def __init__(self, type, value):
        super(Action, self).__init__()
        self.type = type  # 0 is forward/back, 1 is turn
        self.value = value  # How much to change (angular) velocity
