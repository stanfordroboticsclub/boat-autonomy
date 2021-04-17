
class SensorManager(object):
    """Aggregates sim/sensor outputs for processing by robot class"""

    def __init__(self, sim=False, sim_env=None):
        super(SensorManager, self).__init__()
        self.sim = sim
        self.env = sim_env

    def _sim_sensor_readings(self):
        env_state = self.env.get_sensor_observation()
        labels = ["lon", "lat", "speed", "gyro", "magnetometer", "obstacle_data"]
        return dict(zip(labels, env_state))

    def _real_sensor_readings(self):
        return []

    def get_sensor_readings(self):
        if self.sim:
            return self._sim_sensor_readings()
        return self._real_sensor_readings()
