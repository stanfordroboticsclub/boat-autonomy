import numpy as np
from time import sleep, time
from boat_simulation.latlon import LatLon

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

class Robot(object):
    """use robot instead of simple to run on actual robot"""

    def __init__(self, radio=None):
        super(Robot, self).__init__()

        self.waypoints = []

        if radio is not None:
            self.radio = radio
            self.spi = None
        else:
            self.radio = None

            # blindly copied from example code
            # self.spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
            # self.radio = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

        self.last_published = None


    def run(self):
        while True:
            if self.last_published is None or time() - self.last_published > PUBLISH_INTERVAL:
                self.publish_status()
                self.last_published = time()
            self.receive_waypoints()


    def publish_status(self):
        self.transmit_message("just vibing")


    def receive_waypoints(self):
        received = self.receive_packet()
        if received is not None:
            self.waypoints = received


    def receive_packet(self):
        packet = self.radio.receive()
        if (packet is not None): print(f"Received (raw bytes): {packet}")
        return packet


    def transmit_message(self, msg):
        # split into packets of 252 bytes
        packets = [msg[252*i: min(len(msg), i + 252)] for i in range(1 + (len(msg) // 252))]
        for p in packets:
            if p != "":
                self.radio.send(bytes(p, "utf-8"))
