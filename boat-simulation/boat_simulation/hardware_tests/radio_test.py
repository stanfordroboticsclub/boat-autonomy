import numpy as np
from time import sleep, time
from boat_simulation.latlon import LatLon
from boat_simulation.hardware_tests.radio import RadioManager

import digitalio
# import board
import busio
import adafruit_rfm9x

# blindly copied from example code
# likely needs to be modified to correspond to actual wiring
RADIO_FREQ_MHZ = 433.0
# CS = digitalio.DigitalInOut(board.CE1)
# RESET = digitalio.DigitalInOut(board.D25)

PUBLISH_INTERVAL = 1
TIMEOUT = 1

class Robot(object):
    """use robot instead of simple to run on actual robot"""

    def __init__(self, radio=None):
        super(Robot, self).__init__()

        self.waypoints = []

        if radio is not None:
            # blindly copied from example code
            # self.spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
            # self.radio = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

            self.radio = radio
            self.spi = None
        else:
            self.radio = None

        self.radio_manager = RadioManager(self.radio, timeout=TIMEOUT)
        self.last_published = None


    def run(self):
        while True:
            if self.last_published is None or time() - self.last_published > PUBLISH_INTERVAL:
                self.publish_status()
                # print(self.msg_to_packets("very epic very cool"))
                self.last_published = time()
            self.receive_waypoints()


    def publish_status(self):
        self.radio_manager.transmit_message(f"just vibing")


    def receive_waypoints(self):
        received = self.radio_manager.receive_msg()
        if received != None:
            print(f"Received station msg: {received}")
