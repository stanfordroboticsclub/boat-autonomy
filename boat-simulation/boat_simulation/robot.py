import numpy as np
from time import time

from boat_simulation.latlon import LatLon
from boat_simulation.hardware_tests.radio import RadioManager
from boat_simulation.hardware_tests.sensors import SensorManager


TIMEOUT = 1                 # timeout for receiving new waypoints
PUBLISH_INTERVAL = 30 * 60  # publishes status every PUBLISH_INTERVAL seconds
RECEIVE_INTERVAL = 15 * 60  # checks for new waypoints every RECEIVE_INTERVAL seconds


class Robot(object):
    """use robot instead of simple to run on actual robot"""

    def __init__(self, state_estimator, controller, sim=True, base_station_conn=None):
        super(Robot, self).__init__()
        self.sim = sim

        self.state_estimator = state_estimator
        self.controller = controller

        if (sim):
            self.sim_init(base_station_conn)
        else:
            self.robot_init()

        self.set_speed = 0  # speed robot should theoretically be moving at

        self.last_published = None
        self.last_received = None


    def sim_init(self, base_station_conn):
        from boat_simulation.hardware_tests.radio import RadioSim
        from boat_simulation.hardware_tests.sensors import SensorManager

        radio = RadioSim(base_station_conn)
        self.radio_manager = RadioManager(radio, timeout=TIMEOUT)
        self.sensor_manager = SensorManager()


    def robot_init(self):
        import digitalio
        import board
        import busio
        import adafruit_rfm9x

        # set up radio
        RADIO_FREQ_MHZ = 433.0
        CS = digitalio.DigitalInOut(board.CE1)
        RESET = digitalio.DigitalInOut(board.D25)

        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        radio = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)
        radio.tx_power = 23

        self.radio_manager = RadioManager(radio, timeout=TIMEOUT)
        self.sensor_manager = SensorManager()


    def estimate_currents(self):
        return [0, 0]


    def get_raw_state(self):
        readings = self.sensor_manager.get_sensor_readings()
        currents = self.estimate_currents()
        # final [] represents 0 obstacles
        state = [readings["lon"], readings["lat"], readings["speed"], self.set_speed,
            readings["angle"], readings["angular_speed"], []]
        return state


    def execute_action(self, action):
        pass


    def publish_status(self):
        self.radio_manager.transmit_message("{status: just vibing}")


    def receive_waypoints(self):
        received = self.radio_manager.receive_msg()


    def run():
        while True:
            raw_state = self.get_raw_state()
            processed_state = self.state_estimator.estimate(raw_state)

            action = self.controller.choose_action(env, processed_state)
            self.execute_action(action)

            if self.last_published is None or time() - self.last_published > PUBLISH_INTERVAL:
                self.publish_status()
                self.last_published = time()

            if self.last_received is None or time() - self.last_received > RECEIVE_INTERVAL:
                self.receive_waypoints()
                self.last_received = time()
