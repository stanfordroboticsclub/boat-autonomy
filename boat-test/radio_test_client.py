from boat_simulation.hardware_tests.radio_test import Robot
from multiprocessing import Process, Pipe
from boat_simulation.hardware_tests.radio import RadioSim, RadioManager

from time import sleep, time
import argparse


SEND_MSG_INTERVAL = 0.5


def parse_args():
    parser = argparse.ArgumentParser(description='Args for radio test.')
    parser.add_argument('--robot', '-r', help="Set this flag to true if running on robot",
                        action="store_true", default=False)
    args = parser.parse_args()
    return args


def base_station_run(radio_conn):
    last_published = None

    # just to get utils to send/receive msgs as packets
    radio_manager = RadioManager(RadioSim(radio_conn))

    while True:
        if last_published is None or time() - last_published >= SEND_MSG_INTERVAL:
            msg = " ".join(["Hello can you hear me?" for i in range(12)])

            radio_manager.transmit_message(msg)
            last_published = time()

        received_packet = radio_manager.receive_packet()

        if received_packet is not None:
            received_data = radio_manager.extract_packet_data(received_packet)
            print(f"Received robot status: {received_data}")


def robot_run(radio):
    robot = Robot(radio=radio)
    robot.run()


def robot_main():
    print("RUNNING ON ROBOT")

    import digitalio
    import board
    import busio
    import adafruit_rfm9x

    # blindly copied from example code
    # likely needs to be modified to correspond to actual wiring
    RADIO_FREQ_MHZ = 433.0
    CS = digitalio.DigitalInOut(board.CE1)
    RESET = digitalio.DigitalInOut(board.D25)

    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
    # Initialze RFM radio
    radio = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

    radio.tx_power = 23

    robot = Robot(radio=radio)

    robot.run()


def main():
    base_station_conn, radio_conn = Pipe()
    radio = RadioSim(base_station_conn)

    robot_proc = Process(target=robot_run, args=(radio,))

    try:
        robot_proc.start()
        base_station_run(radio_conn)
    finally:
        robot_proc.terminate()


if __name__ == '__main__':
    args = parse_args()

    if args.robot:
        robot_main()
    else:
        main()
