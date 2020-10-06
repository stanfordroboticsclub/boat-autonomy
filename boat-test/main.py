from boat_simulation.simple import SimpleBoatSim
from controller.keyboard_controller import KeyboardController
from controller.autonomy_controller_template import AutonomyControllerTemplate
import argparse


def main():
    # Command line arguments
    controller_arg_names = ["keyboard", "autonomy_template"]

    parser = argparse.ArgumentParser(description='Run the boat simulation.')
    parser.add_argument('--controller', '-c', help="Choose the name of the controller to use",
                        choices=controller_arg_names, default=controller_arg_names[0])
    args = parser.parse_args()

    env = SimpleBoatSim()
    state = env.reset()

    controller = None
    if args.controller == "keyboard":
        controller = KeyboardController()
    elif args.controller == "autonomy_template":
        controller = AutonomyControllerTemplate()

    print("Instantiated controller:", controller.name)
    
    while True:
        action = controller.choose_action(env, state)
        state, _, end_sim, _ = env.step(action)[0]
        env.render()
        
        if end_sim:
            # This can be replaced with env.close() to end the simulation.
            env.reset()


if __name__ == '__main__':
    main()
