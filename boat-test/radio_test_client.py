from boat_simulation.hardware_tests.radio_test import Robot
from multiprocessing import Process, Pipe
from boat_simulation.hardware_tests.radio_simulator import RadioSim, RadioManager

from time import sleep, time


SEND_MSG_INTERVAL = 0.5


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
    main()
