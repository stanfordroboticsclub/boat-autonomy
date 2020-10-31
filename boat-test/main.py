from boat_simulation.simple import SimpleBoatSim
from controller.keyboard_controller import KeyboardController
from controller.autonomy_controller_template import AutonomyControllerTemplate
from controller.complementary_filter import ComplementaryFilterController
from controller.minimal_controller import MinimalController

import argparse


def parse_args():
    controller_arg_names = ["keyboard", "autonomy_template", "complementary_filter_test", "minimal_controller"]
    state_modes = ["ground_truth", "noisy", "sensor"]

    parser = argparse.ArgumentParser(description='Run the boat simulation.')
    parser.add_argument('--controller', '-c', help="Choose the name of the controller to use",
                        choices=controller_arg_names, default=controller_arg_names[0])
    parser.add_argument('--current_level', '-cl', help="Choose the intensity of currents in the simulation",
                        default=3)
    parser.add_argument('--max_obstacles', '-mo', help="Choose the maximum number of obstacles on screen at any time",
                        default=10)
    parser.add_argument('--state_mode', '-sm', help="Choose the representation of the simulation state available to the boat",
                        choices=state_modes, default=state_modes[0])
    parser.add_argument('--no_render', '-nr', help="Set this flag in order to render the simulation",
                        action="store_true", default=False)
    args = parser.parse_args()
    return args


def main():
    args = parse_args()
    env = SimpleBoatSim(current_level=int(args.current_level), state_mode=args.state_mode, max_obstacles=int(args.max_obstacles))
    state = env.reset()

    controller = None
    if args.controller == "keyboard":
        controller = KeyboardController()
    elif args.controller == "autonomy_template":
        controller = AutonomyControllerTemplate()
    elif args.controller == "complementary_filter_test":
        controller = ComplementaryFilterController()
    elif args.controller == "minimal_controller":
        controller = MinimalController()

    print("Instantiated controller:", controller.name)

    while True:
        action = controller.choose_action(env, state)
        state, _, end_sim, _ = env.step(action)

        if not args.no_render:
            env.render()

        if end_sim:
            # This can be replaced with env.close() to end the simulation.
            env.reset()

if __name__ == '__main__':
    main()
