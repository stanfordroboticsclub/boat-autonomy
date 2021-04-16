from boat_simulation.latlon import LatLon
from boat_simulation.robot import Robot
from boat_simulation.managers.radio import RadioSim, RadioManager

from time import sleep, time
import argparse
from multiprocessing import Process, Pipe

from controller.keyboard_controller import KeyboardController
from controller.autonomy_controller_template import AutonomyControllerTemplate
from controller.minimal_controller import MinimalController
from controller.scipy_opt_controller import ScipyOptController
from controller.scipy_logging_controller import ScipyLoggingController
from controller.pid_controller import PIDController
from controller.slsqp_controller import SLSQPController
from controller.planning_controller import PlanningController
from controller.voronoi_planning_controller import VoronoiPlanningController
from controller.control_planner import ControlPlanner
from controller.state_estimation_test import SETestController

from state_estimators.base_estimator import IdentityEstimator
from state_estimators.complementary_filter import ComplementaryFilter

SEND_MSG_INTERVAL = 0.5


def parse_args():
    controller_arg_names = ["keyboard", "autonomy_template", "complementary_filter_test", "minimal_controller", "scipy_logging",
        "scipy_opt", "pid", "slsqp", "planning", "voronoi_planning", "c_planning", "se_test"]
    state_modes = ["ground_truth", "noisy", "sensor"]

    parser = argparse.ArgumentParser(description='Args for hardware test.')
    parser.add_argument('--robot', '-r', help="Set this flag to true if running on robot",
                        action="store_true", default=False)
    parser.add_argument('--controller', '-c', help="Choose the name of the controller to use",
                        choices=controller_arg_names, default=controller_arg_names[0])
    parser.add_argument('--current_level', '-cl', help="Choose the intensity of currents in the simulation in cm/s",
                        default=50)
    parser.add_argument('--max_obstacles', '-mo', help="Choose the maximum number of obstacles on screen at any time",
                        default=10)
    parser.add_argument('--state_mode', '-sm', help="Choose the representation of the simulation state available to the boat",
                        choices=state_modes, default=state_modes[2])
    parser.add_argument('--no_render', '-nr', help="Set this flag to true to disable rendering the simulation",
                        action="store_true", default=False)
    parser.add_argument('--no_drag', '-nd', help="Set this flag to true to disable drag forces",
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


def robot_run(state_estimator, controller, base_station_conn, args):
    robot = Robot(state_estimator, controller, sim=True,
        base_station_conn=base_station_conn, args=args)
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


def main(args):
    base_station_conn, radio_conn = Pipe()
    radio = RadioSim(base_station_conn)

    state_estimator = ComplementaryFilter(0.9)

    controller = None
    if args.controller == "keyboard":
        controller = KeyboardController()
    elif args.controller == "autonomy_template":
        controller = AutonomyControllerTemplate()
    elif args.controller == "complementary_filter_test":
        controller = KeyboardController()
        state_estimator = ComplementaryFilter(0.9, to_print=True)
    elif args.controller == "minimal_controller":
        controller = MinimalController()
    elif args.controller == "scipy_logging":
        controller = ScipyLoggingController()
    elif args.controller == "scipy_opt":
        controller = ScipyOptController()
    elif args.controller == "pid":
        controller = PIDController()
    elif args.controller == "slsqp":
        controller = SLSQPController()
    elif args.controller == "planning":
        controller = PlanningController()
    elif args.controller == "voronoi_planning":
        controller = VoronoiPlanningController()
    elif args.controller == "c_planning":
        controller = ControlPlanner()
    elif args.controller == "se_test":
        controller = SETestController()
    print("Instantiated controller:", controller.name)

    robot_proc = Process(target=robot_run, args=(state_estimator, controller, base_station_conn, args))

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
        main(args)
