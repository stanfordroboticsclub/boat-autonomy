from boat_simulation.simple import SimpleBoatSim
from boat_simulation.latlon import LatLon

from controller.keyboard_controller import KeyboardController
from controller.autonomy_controller_template import AutonomyControllerTemplate
from controller.complementary_filter import ComplementaryFilterController
from controller.minimal_controller import MinimalController
from controller.scipy_opt_controller import ScipyOptController
from controller.scipy_logging_controller import ScipyLoggingController
from controller.xy_controller import XYController
from controller.slsqp_controller import SLSQPController

from multiprocessing import Process
from multiprocessing.connection import Listener
from multiprocessing.connection import Client

import argparse
import pygame
import types
import json


def parse_args():
    controller_arg_names = ["keyboard", "autonomy_template", "complementary_filter_test", "minimal_controller", "scipy_logging", "scipy_opt", "xy", "slsqp"]
    state_modes = ["ground_truth", "noisy", "sensor"]

    parser = argparse.ArgumentParser(description='Run the boat simulation.')
    parser.add_argument('--controller', '-c', help="Choose the name of the controller to use",
                        choices=controller_arg_names, default=controller_arg_names[0])
    parser.add_argument('--current_level', '-cl', help="Choose the intensity of currents in the simulation in cm/s",
                        default=50)
    parser.add_argument('--max_obstacles', '-mo', help="Choose the maximum number of obstacles on screen at any time",
                        default=10)
    parser.add_argument('--state_mode', '-sm', help="Choose the representation of the simulation state available to the boat",
                        choices=state_modes, default=state_modes[0])
    parser.add_argument('--no_render', '-nr', help="Set this flag in order to render the simulation",
                        action="store_true", default=False)
    args = parser.parse_args()
    return args


def format_state(state, env):
    boat_x, boat_y, boat_speed, boat_angle, boat_ang_vel, obstacles = state
    currents = env.compute_ocean_current(LatLon(boat_y, boat_x))
    out_dict = {
        "state": {
            "lat": boat_y,
            "lon": boat_x,
            "speed": boat_speed,
            "angle": boat_angle,
            "ang_vel": boat_ang_vel,
            "ocean_current_x": currents[0],
            "ocean_current_y": currents[1],
            "desired_speed": env.speed,
            "obstacles": obstacles
        }
    }
    return json.dumps(out_dict)


def simulation(args):
    """
    Simulates movements of the boat.

    This function is to be executed in the process simulates the boat. It creates
    an instance of the SimpleBoatSim class, repeatedly publishes state
    information and receives actions taken by the boat.
    """
    address = ('localhost', 8000)     # family is deduced to be 'AF_INET'
    listener = Listener(address, authkey=b'secret password')
    conn = listener.accept()

    env = SimpleBoatSim(current_level=int(args.current_level), state_mode=args.state_mode, max_obstacles=int(args.max_obstacles))
    state = env.reset()

    env.set_waypoints(conn.recv())
    conn.send(format_state(state, env))

    while True:
        action = conn.recv()
        state, _, end_sim, _ = env.step(action)

        if not args.no_render:
            env.render()

        if end_sim:
            # This can be replaced with env.close() to end the simulation.
            state = env.reset()

        conn.send(format_state(state, env))


def controller(args):
    """
    Specifies linear and angular accelerations to be applied by boat.

    This function is to be executed in the process that handles (or simulates
    handling) the main control of the boat.
    """
    controller = None
    if args.controller == "keyboard":
        controller = KeyboardController(in_sim=False)
    elif args.controller == "autonomy_template":
        controller = AutonomyControllerTemplate(in_sim=False)
    elif args.controller == "complementary_filter_test":
        controller = ComplementaryFilterController(in_sim=False)
    elif args.controller == "minimal_controller":
        controller = MinimalController(in_sim=False)
    elif args.controller == "scipy_logging":
        controller = ScipyLoggingController(in_sim=False)
    elif args.controller == "scipy_opt":
        controller = ScipyOptController(in_sim=False)
    elif args.controller == "xy":
        controller = XYController(in_sim=False)
    elif args.controller == "slsqp":
        controller = SLSQPController(in_sim=False)

    address = ('localhost', 8000)
    conn = Client(address, authkey=b'secret password')

    radio_address = ('localhost', 8001)
    radio_conn = Client(radio_address, authkey=b'secret password')

    waypoints = radio_conn.recv()
    conn.send(waypoints)
    env = types.SimpleNamespace(waypoints=waypoints)

    while True:
        state = json.loads(conn.recv())["state"]
        state = state["lon"], state["lat"], state["speed"], state["desired_speed"], state["angle"], state["ang_vel"], state["ocean_current_x"], state["ocean_current_y"], state["obstacles"]

        action = controller.choose_action(env, state)
        conn.send(action)


def radio(args):
    """
    Sends waypoint information to the controller program.

    This function is to be executed in the process that processes (or simulates
    processing) radio input from the dashboard to the controller that specifies
    where the robot is supposed to go.
    """
    address = ('localhost', 8001)     # family is deduced to be 'AF_INET'
    listener = Listener(address, authkey=b'secret password')
    conn = listener.accept()

    env = SimpleBoatSim(current_level=int(args.current_level), state_mode=args.state_mode, max_obstacles=int(args.max_obstacles))
    state = env.reset()

    conn.send(env.waypoints)


def main():
    args = parse_args()
    radio_proc = Process(target=radio, args=(args,))
    simulation_proc = Process(target=simulation, args=(args,))
    controller_proc = Process(target=controller, args=(args,))

    try:
        radio_proc.start()
        simulation_proc.start()
        controller_proc.start()

        while True:
            pass

    finally:
        simulation_proc.terminate()
        controller_proc.terminate()
        radio_proc.terminate()


if __name__ == '__main__':
    main()
