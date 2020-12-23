from boat_simulation.simple import SimpleBoatSim
from boat_simulation.latlon import LatLon

from controller.keyboard_controller import KeyboardController
from controller.autonomy_controller_template import AutonomyControllerTemplate
from controller.complementary_filter import ComplementaryFilterController
from controller.minimal_controller import MinimalController
from controller.scipy_opt_controller import ScipyOptController
from controller.scipy_logging_controller import ScipyLoggingController
from controller.pid_controller import PIDController
from controller.slsqp_controller import SLSQPController

import pandas as pd
import argparse


def parse_args():
    controller_arg_names = ["keyboard", "autonomy_template", "complementary_filter_test", "minimal_controller", "scipy_logging", "scipy_opt", "pid", "slsqp"]
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
    return boat_x, boat_y, boat_speed, env.speed, boat_angle, boat_ang_vel, currents[0], currents[1], obstacles


def test_controller(controller, env, args):
    num_trials = 5
    curr_trial = 0

    trial_avg_a = 0
    trial_avg_alpha = 0
    trial_avg_t = 0

    avg_a = 0
    avg_alpha = 0
    avg_t = 0

    while curr_trial < num_trials:
        finished_first = False
        state = env.reset()

        num_t = 0
        last_waypoint = 0
        last_t = 0

        print(f"Trial: {curr_trial}")

        while True:
            if controller.curr_waypoint != 0 and not finished_first:
                finished_first = True
            elif controller.curr_waypoint == 0 and finished_first:
                curr_trial += 1
                break

            action = controller.choose_action(env, format_state(state, env))

            if action.type == 2:
                if last_t == 0:
                    last_t = num_t
                if last_waypoint != controller.curr_waypoint:
                    time_taken = (num_t - last_t) / 60
                    last_t = num_t
                    trial_avg_t = ((controller.curr_waypoint * trial_avg_t) + time_taken) / (1 + controller.curr_waypoint)
                    last_waypoint = controller.curr_waypoint

                vals = action.value
                trial_avg_a = ((num_t*trial_avg_a) + abs(vals[1])) / (num_t + 1)
                trial_avg_alpha = ((num_t*trial_avg_alpha) + abs(vals[0])) / (num_t + 1)
                num_t += 1

            state, _, end_sim, _ = env.step(action)

            if not args.no_render:
                env.render()

            if end_sim:
                # This can be replaced with env.close() to end the simulation.
                env.reset()

        avg_a = ((curr_trial*avg_a) + trial_avg_a) / (curr_trial + 1)
        avg_alpha = ((curr_trial*avg_alpha) + trial_avg_alpha) / (curr_trial + 1)
        avg_t = ((curr_trial*avg_t) + trial_avg_t) / (curr_trial + 1)

    return avg_a, avg_alpha, avg_t


def main():
    args = parse_args()
    env = SimpleBoatSim(current_level=int(args.current_level), state_mode=args.state_mode, max_obstacles=int(args.max_obstacles))
    state = env.reset()

    controllers = [MinimalController(print_info=False), PIDController(print_info=False), SLSQPController(print_info=False)]

    cols = "name, avg_a, avg_alpha, avg_time_to_waypoint".split(", ")
    results = pd.DataFrame(columns=cols)

    for controller in controllers:
        print(f"Testing: {controller.name}")
        avg_a, avg_alpha, avg_time_to_waypoint = test_controller(controller, env, args)
        res_dict = dict(zip(cols, [controller.name, avg_a, avg_alpha, avg_time_to_waypoint]))
        results = results.append(res_dict, ignore_index=True)

    results.to_csv("logs/results.csv")


if __name__ == '__main__':
    main()
