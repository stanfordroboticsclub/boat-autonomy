from time import time
import numpy as np


BOAT_WIDTH = 0.5            # meters
BOAT_HEIGHT = 1             # meters
BOAT_MASS = 5               # kg
MAX_FORCE = 49              # newtons

class MotorManager(object):
    """Controls real motors and executes actions in sim"""

    def __init__(self, sim=False, sim_env=None):
        super(MotorManager, self).__init__()
        self.sim = sim
        self.env = sim_env

        if not self.sim: self.robot_init()


    def robot_init(self):
        import pigpio

        self.pi = pigpio.pi()

        # Initialize ESC
        self.left_esc = 12
        self.right_esc = 13

        for esc_pin in [self.left_esc, self.right_esc]:
            # 100 Hz -> period of 0.01sec = 10000 microsec
            # need on for 1500 microsec -> 1500/10000 = 15% duty cycle
            start_time = time()
            self.pi.hardware_PWM(esc_pin, 100, 0.15 * 1000000)

            while time() - start_time < 3:
                continue

            self.pi.hardware_PWM(esc_pin, 0, 0)


    def _sim_take_action(self, action):
        self.env.step(action)


    def _real_take_action(self, action):
        accel, alpha = action

        # compute thrusts to apply
        a_term = accel * BOAT_MASS / 2
        alpha_term = (BOAT_MASS * BOAT_HEIGHT**2 * alpha) / (3 * BOAT_WIDTH)

        F_1 = a_term + alpha_term   # port
        F_2 = a_term - alpha_term   # starboard

        np.clip(F_1, -MAX_FORCE, MAX_FORCE)
        np.clip(F_2, -MAX_FORCE, MAX_FORCE)

        t_1 = (400/MAX_FORCE) * F_1
        t_2 = (400/MAX_FORCE) * F_2

        self.pi.hardware_PWM(self.left_esc, 100, (t_1 / 10000) * 1000000)
        self.pi.hardware_PWM(self.right_esc, 100, (t_2 / 10000) * 1000000)



    def take_action(self, action):
        if self.sim:
            return self._sim_take_action(action)
        return self._real_take_action(action)
