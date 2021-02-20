from time import time

class RadioSim(object):
    """simulates a radio"""

    def __init__(self, station_conn):
        super(RadioSim, self).__init__()
        self.station_conn = station_conn

    def send(self, msg):
        self.station_conn.send(msg)

    def receive(self):
        if self.station_conn.poll():
            return self.station_conn.recv()
        return None


class RadioManager(object):
    """provides higher level functionality for radio"""

    def __init__(self, radio, timeout=1):
        super(RadioManager, self).__init__()
        self.radio = radio
        self.message_num = 0
        self.timeout = timeout


    def transmit_message(self, msg):
        # split into packets of 252 bytes
        packets = self.msg_to_packets(msg);
        for p in packets:
            if p != "":
                self.radio.send(p)


    def msg_to_packets(self, msg, data_size=250):
        msg = bytes(msg, "utf-8")

        # pad to a multiple of 250
        msg = msg + bytes([0 for i in range(250 - (len(msg) % 250))])

        sub_msgs = [msg[data_size*i: min(len(msg), data_size*i + data_size)] for i in range(1 + (len(msg) // data_size))]
        if sub_msgs[-1] == b'': sub_msgs.pop(len(sub_msgs)-1)

        packets = []

        for i in range(len(sub_msgs)):
            sub_msg = sub_msgs[i]

            packet = b''
            packet += bytes([self.message_num])
            packet += bytes([(i << 1) + (1 if (i == len(sub_msgs) - 1) else 0)])
            packet += sub_msg
            packets.append(packet)

        self.message_num += 1
        self.message_num %= 256

        return packets


    def receive_msg(self):
        bytes = b''
        curr_t = time()

        while True:
            if time() - curr_t > self.timeout:
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


    def receive_packet(self):
        packet = self.radio.receive()
        return packet


    def extract_packet_data(self, packet):
        for k in range(2, len(packet)):
            if packet[k] == 0:
                break

        return packet[2: k]


    def process_packet(self, packet):
        processed = self.extract_packet_data(packet)
        return processed, bool(packet[1] & 1)
