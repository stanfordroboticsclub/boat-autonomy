# boat-autonomy

## Usage

### Regular use

Two different interfaces are provided to interact with the robot/simulation. The first is an OpenAI Gym style environment which can be used to test algorithms for boat autonomy, and the second uses a 'Robot' class to allow switching between simulation and hardware more easily. To use either, first install the environment:

- Download the folder named ‘boat’
- (Optionally install anaconda, create a venv, and activate it)
- `cd boat-simulation`
- `pip install -e .`
- `cd ../boat-test`

To run the program:

- If you want to run using the OpenAI Gym style environment, `python main.py`. Otherwise, `python robot_run.py`. Add `-h` to see all available options
- To specify a controller to use, use the flag `-c controller_name`. When not specified, it defaults to  `KeyboardController`.
- The intensity of the current can be controlled using the `-cl` flag, eg: `python main.py -cl 30`. Default is 50, and the units are in cm/s.
- The nature of the state representation available to the controller to plan its actions can be controlled with the `-sm` flag, eg: `python main.py -sm noisy`. This only works with `main.py` and not `robot_run.py`. Default is `ground_truth`.

### Multiprocessing

- `message_passing_run.py` uses the Client and Listener features of the `multiprocessing.connection` module to separate the environment, the controller, and the 'radio' that sets waypoints into different processes.
- `pipe_run.py` uses `multiprocessing`'s Pipe functionality for ipc to do the same

Both files can be run exactly like `main.py` with the same flags.

### More Info

If you encounter any issues with running the simulation, refer to the troubleshooting section at the end of this document.

The simulation library is modeled after OpenAI Gym and uses Pygame. It provides a class SimpleBoatSim in `simple.py` that you use to simulate the environment. The way you use SimpleBoatSim is shown in `boat-test/main.py`; you need an outer loop that repeatedly calls `step(some action)` and `render()` (if you want to visualize the simulation). `step(action)` returns a state, a reward (currently always 0), whether the simulation terminated, and some extra info (currently just `None`).

The `Robot` class essentially wraps the environment if running in simulation, but can be made to use actual hardware if available by setting the `-r` flag. In order to make switching between the simulated environment and the real world as seamless as possible, it abstracts sensor, radio, and motor functionality into several 'managers' that behave differently depending on whether the program is running in simulation or on the actual robot; these managers can be found in `boat_simulation/managers`.

### Controllers

| Controller             | ID                          | Description                                                                                                                            |
|------------------------|-----------------------------|----------------------------------------------------------------------------------------------------------------------------------------|
| KeyboardController     | `keyboard_controller`       | Move the boat with your the arrow keys of your keyboard.                                                                               |
| ComplementaryFilter    | `complementary_filter_test` | Provides an experimental implementation of a complementary filter to estimate the state of the boat. For use with `sensor` state mode. |
| SLSQPController        | `slsqp`                     | Uses SLSQP to optimize distance from destination after a dynamically adjusted window. For use with `ground_truth` state mode.          |
| MinimalController      | `minimal_controller`        | Proof of concept controller that follows generated path. Works alright for small currents. For use with `ground_truth` state mode.     |
| ScipyOptController     | `scipy_opt`                 | Experimental controller using SciPy optimization library to optimize objective function.                                               |
| ScipyLoggingController | `scipy_logging`             | Same as `ScipyOptController`, but logs parameters and intermediate values to a file and stops at the first waypoint.                   |

### State Representation

The `ground_truth` state is encoded as a list:

```
[boat x, boat y, boat speed, boat angle, boat angular velocity,
    [
        [obstacle 1 radius, obstacle 1 x, obstacle 1 y, obstacle 1 x-velocity, obstacle 1 y-velocity],
        [obstacle 2 radius, ...],
        ...
    ]
]
```

The elements of this list can be used by the robot to autonomously plan its path and avoid obstacles. The `noisy` state mode returns the same but with some random noise added to all fields (except the destination). Currently, `sensor` state mode only returns: `[angular_velocity, heading]` to simulate the output of a gyro and magnetometer respectively. Both these readings are noisy.

### Writing autonomy code

There are a few steps that are necessary to write custom autonomy code.

1. Duplicate the `autonomy_controller_template.py` file in `boat-test/controller`.
2. Fill in the `select_action_from_state(env, state)` method with the custom autonomy code.
3. In `main.py`, add a short identifier for the new controller into the `controller_arg_names` list.
4. Add a new `elif` statement in the section for setting the controller. Follow the pattern of the previous `if`/`elif` statements, replacing the identifier string with the custom identifier from step 2.
5. Run the simulation with `python main.py -c custom_identifier`.

## Todos

- More work on state estimation from noisy input, integrate with controllers
- Add boundaries to all controllers so that the boat doesn't go off very far from the screen
- Local map on main screen + minimap on top right showing global position so that the boat isn't extremely tiny on the main screen

## Troubleshooting

**Pygame running slowly on macOS**

If this problem occurs, install Pygame version 2.0.0.dev6 as follows:

`pip install pygame==2.0.0.dev6`

Once it is installed, the simulation should run much more smoothly.
