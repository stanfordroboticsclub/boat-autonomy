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
TIMEOUT = 1

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
        self.message_num = 0


    def run(self):
        while True:
            if self.last_published is None or time() - self.last_published > PUBLISH_INTERVAL:
                self.publish_status()
                # print(self.msg_to_packets("very epic very cool"))
                self.last_published = time()
            self.receive_waypoints()


    def publish_status(self):
        self.transmit_message(f"just vibing")


    def msg_to_packets(self, msg, data_size=250):
        msg = bytes(msg, "utf-8")

        if len(msg) > 250:
            msg = msg[0:250]
        elif len(msg) < 250:
            msg = msg + bytes([0 for i in range(250 - len(msg))])

        sub_msgs = [msg[data_size*i: min(len(msg), data_size*i + data_size)] for i in range(1 + (len(msg) // data_size))]
        packets = []

        for i in range(len(sub_msgs)):
            sub_msg = sub_msgs[i]

            if sub_msg != b'':
                packet = b''
                packet += bytes([self.message_num])
                packet += bytes([(i << 1) + 1 if i == len(sub_msgs) - 1 else 0])
                packet += sub_msg
                packets.append(packet)

        self.message_num += 1
        self.message_num %= 256

        return packets


    def transmit_message(self, msg):
        # split into packets of 252 bytes
        packets = self.msg_to_packets(msg);
        for p in packets:
            if p != "":
                self.radio.send(p)


    def receive_waypoints(self):
        received = self.receive_msg()
        if len(received) != 0: print(f"Received station msg: {received}")
        if received is not None:
            self.waypoints = received


    def receive_packet(self):
        packet = self.radio.receive()
        return packet


    def process_packet(self, packet):
        processed = b''
        for byte in packet:
            byte = bytes([byte])
            if byte == b'\4':  # EOT
                return processed, True
            else:
                processed += byte
        return processed, False


    def receive_msg(self):
        bytes = b''
        curr_t = time()

        while True:
            if time() - curr_t > TIMEOUT:
                print("TIMEOUT")
                bytes = b''
                break

            packet = self.receive_packet()

            if packet is not None:
                new_bytes, eot = self.process_packet(packet)
                bytes += new_bytes
                curr_t = time()
                if eot: break

        return bytes
