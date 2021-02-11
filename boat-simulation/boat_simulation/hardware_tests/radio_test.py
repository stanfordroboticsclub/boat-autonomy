import numpy as np
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

class Robot(object):
    """use robot instead of simple to run on actual robot"""

    def __init__(self):
        super(Robot, self).__init__()

        # blindly copied from example code
        # self.spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        # self.rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)


    def transmit_message(self, msg):
        # split into packets of 252 bytes
        packets = [msg[252*i: min(len(msg), i + 252)] for i in range(1 + (len(msg) // 252))]
        for p in packets:
            if p != "":
                print(f"{p}, {len(p.encode('utf-8'))}")
                # rfm9x.send(bytes(p, "utf-8"))
