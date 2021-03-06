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

    def __init__(self, state_estimator, controller, sim=True, base_station_conn=None, args=None):
        super(Robot, self).__init__()
        self.sim = sim
        self.sim_env = None
        self.args = args

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
        from boat_simulation.simple import SimpleBoatSim
        from boat_simulation.hardware_tests.radio import RadioSim
        from boat_simulation.hardware_tests.sensors import SensorManager

        self.sim_env = SimpleBoatSim(current_level=int(self.args.current_level), state_mode=self.args.state_mode,
            max_obstacles=int(self.args.max_obstacles), apply_drag_forces=(not bool(self.args.no_drag)))
        self.sim_env.reset()

        radio = RadioSim(base_station_conn)
        self.radio_manager = RadioManager(radio, timeout=TIMEOUT)
        self.sensor_manager = SensorManager(sim=True, sim_env=self.sim_env)


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


    def execute_action(self, action):
        pass


    def publish_status(self):
        self.radio_manager.transmit_message("{status: just vibing}")


    def receive_waypoints(self):
        received = self.radio_manager.receive_msg()


    def format_state(self, state):
        boat_x, boat_y, boat_speed, boat_angle, boat_ang_vel, obstacles = state
        currents = self.estimate_currents()
        return boat_x, boat_y, boat_speed, boat_speed, boat_angle, boat_ang_vel, currents[0], currents[1], obstacles


    def run(self):
        while True:
            raw_state = self.sensor_manager.get_sensor_readings()
            processed_state = self.format_state(self.state_estimator.estimate(raw_state))

            # BIG PROBLEM HERE: CONTROLLER RELIES ON SIM ENV TO SELECT ACTION
            action = self.controller.choose_action(self.sim_env, processed_state)
            self.execute_action(action)

            if self.last_published is None or time() - self.last_published > PUBLISH_INTERVAL:
                self.publish_status()
                self.last_published = time()

            if self.last_received is None or time() - self.last_received > RECEIVE_INTERVAL:
                self.receive_waypoints()
                self.last_received = time()

            if self.sim and not self.args.no_render:
                self.sim_env.render()
