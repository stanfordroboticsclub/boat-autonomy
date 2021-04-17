BOAT_WIDTH = 0.5    # meters
BOAT_HEIGHT = 1     # meters
BOAT_MASS = 5       # kg

class MotorManager(object):
    """Controls real motors and executes actions in sim"""

    def __init__(self, sim=False, sim_env=None):
        super(MotorManager, self).__init__()
        self.sim = sim
        self.env = sim_env

    def _sim_take_action(self, action):
        self.env.step(action)

    def _real_take_action(self, action):
        accel, alpha = action

        # compute thrusts to apply
        a_term = accel * BOAT_MASS / 2
        alpha_term = (BOAT_MASS * BOAT_HEIGHT**2 * alpha) / (3 * BOAT_WIDTH)

        F_1 = a_term + alpha_term   # port
        F_2 = a_term - alpha_term   # starboard

        # TODO: apply thrusts

    def take_action(self, action):
        if self.sim:
            return self._sim_take_action(action)
        return self._real_take_action(action)
