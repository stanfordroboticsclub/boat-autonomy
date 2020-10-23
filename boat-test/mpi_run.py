from mpi4py import MPI
import argparse
import pygame

from pygame.locals import KEYDOWN

from boat_simulation.simple import SimpleBoatSim, Action
from controller.keyboard_mpi import KeyboardMPI
from controller.autonomy_controller_template import AutonomyControllerTemplate
from controller.complementary_filter import ComplementaryFilterController


def parse_args():
    controller_arg_names = ["keyboard", "autonomy_template", "complementary_filter_test"]
    state_modes = ["ground_truth", "noisy", "sensor", "pygame"]

    parser = argparse.ArgumentParser(description='Run the boat simulation.')
    parser.add_argument('--controller', '-c', help="Choose the name of the controller to use",
                        choices=controller_arg_names, default=controller_arg_names[0])
    parser.add_argument('--current_level', '-cl', help="Choose the intensity of currents in the simulation",
                        default=3)
    parser.add_argument('--state_mode', '-sm', help="Choose the representation of the simulation state available to the boat",
                        choices=state_modes, default=state_modes[0])
    parser.add_argument('--no_render', '-nr', help="Set this flag in order to render the simulation",
                        action="store_true", default=False)
    args = parser.parse_args()
    return args


def environment_loop(args, comm, rank):
    env = SimpleBoatSim(current_level=int(args.current_level), state_mode=args.state_mode)
    state = env.reset()
    req = comm.isend(state, dest=1, tag=2)
    req.wait()

    while True:
        if args.controller == "keyboard":
            events = [e for e in pygame.event.get()]
            keydowns = [event.key for event in events if event.type == KEYDOWN]

            env_req = comm.isend(keydowns, dest=1, tag=3)
            env_req.wait()

        action_req = comm.irecv(source=1, tag=1)
        action_data = action_req.wait()
        action = Action(action_data[0], action_data[1])
        state, _, end_sim, _ = env.step(action)

        if end_sim:
            state = env.reset()

        if not args.no_render:
            env.render()

        state_req = comm.isend(state, dest=1, tag=2)
        state_req.wait()


def controller_loop(args, comm, rank):
    pygame.init()
    controller = None
    if args.controller == "keyboard":
        controller = KeyboardMPI()
    elif args.controller == "autonomy_template":
        controller = AutonomyControllerTemplate()
    elif args.controller == "complementary_filter_test":
        controller = ComplementaryFilterController()
    print("Instantiated controller:", controller.name)

    while True:
        state_req = comm.irecv(source=0, tag=2)
        state = state_req.wait()
        env = ""

        if args.controller == "keyboard":
            env_req = comm.irecv(source=0, tag=3)
            env = env_req.wait()

        action = controller.choose_action(env, state)

        action_req = comm.isend([action.type, action.value], dest=0, tag=1)
        action_req.wait()

def main():
    comm = MPI.COMM_WORLD
    rank = comm.Get_rank()
    args = parse_args()

    if rank == 0:       # environment rank
        print("here 0")
        environment_loop(args, comm, rank)

    elif rank == 1:     # autonomy rank
        print("here 1")
        controller_loop(args, comm, rank)


if __name__ == '__main__':
    main()
