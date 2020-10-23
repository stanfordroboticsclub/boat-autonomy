# boat-autonomy

## Usage

### Regular Usage

- Download the folder named ‘boat’
- (Optionally install anaconda, create a venv, and activate it)
- `cd boat-simulation`
- `pip install -e .`
- `cd ../boat-test`
- `python main.py` (Add the flag `--no_render` or `-nr` if you do not want to visualize the simulation)
- To specify a controller to use, type `python main.py -c controller_name`. When not specified, it defaults to the KeyboardController.
- The intensity of the current can be controlled using the `-cl` flag, eg: `python main.py -cl 5`. Default is 3.
- The nature of the state representation available to the controller to plan its actions can be controlled with the `-sm` flag, eg: `python main.py -sm noisy`.
  Default is `ground_truth`.
- A full list of controller names and all other arguments can be accessed in the help entry (`python main.py -h`)

If you encounter any issues with running the simulation, refer to the troubleshooting section at the end of this document.

The simulation library is modeled after OpenAI Gym and uses Pygame. Essentially, it provides a class SimpleBoatSim in `simple.py` that you use to simulate the environment. The way you use SimpleBoatSim is shown in `boat-test/main.py`; you need an outer loop that repeatedly calls `step(some action)` and `render()` (if you want to visualize the simulation). `step(action)` returns a state, a reward, whether the simulation terminated, and any other info. The state is encoded as a list:

```
[boat x, boat y, boat speed, boat angle, boat angular velocity,
    [
        [obstacle 1 radius, obstacle 1 x, obstacle 1 y, obstacle 1 x-velocity, obstacle 1 y-velocity],
        [obstacle 2 radius, ...],
        ...
    ]
]
```

The elements of this list can be used by the robot to autonomously plan its path and avoid obstacles.

### MPI Usage

The environment and the autonomy code can be run as two separate processes using MPI. In order to do this:
- Install MPI and mpi4py as described here `https://mpi4py.readthedocs.io/en/stable/install.html`
- Run using `mpiexec -n 2 python mpi_run.py`. You can add extra options through the flags described in the "regular usage" section.

### Writing autonomy code

There are a few steps that are necessary to write custom autonomy code.

1. Duplicate the `autonomy_controller_template.py` file in `boat-test/controller`.
2. Fill in the `select_action_from_state(env, state)` method with the custom autonomy code.
3. In `main.py`, add a short identifier for the new controller into the `controller_arg_names` list.
4. Add a new `elif` statement in the section for setting the controller. Follow the pattern of the previous `if`/`elif` statements, replacing the identifier string with the custom identifier from step 2.
5. Run the simulation with `python main.py -c custom_identifier`.

## Todos

- Detect collision of boat with obstacle and terminate simulation: `CollisionDetection` branch
- Add currents
- More diverse obstacles
- Change the state representation (for example don’t give exact coordinates but distances to nearby obstacles)
- Change what the actions do? (right now increase/decrease angular velocity + linear velocity)
- Make all the magic constants used actually align with physical units
- Add noise to state to simulate how estimates from sensors won’t be perfect
- Actual autonomy using the simulation
- Refactor code to make more clean, cut any inefficiencies, use pygame best practices
- Probably a lot more

## Troubleshooting

**Pygame running slowly on macOS**

If this problem occurs, install Pygame version 2.0.0.dev4 as follows:

`pip install pygame==2.0.0.dev6`

Once it is installed, the simulation should run much more smoothly.
