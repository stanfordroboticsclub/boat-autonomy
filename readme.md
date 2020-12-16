# boat-autonomy

## Usage

### Regular use

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

### Multiprocessing

- `message_passing_run.py` uses the Client and Listener features of the `multiprocessing.connection` module to separate the environment, the controller, and the 'radio' that sets waypoints into different processes.
- `pipe_run.py` uses `multiprocessing`'s Pipe functionality for ipc to do the same

Both files can be run exactly like `main.py` with the same flags.

### More Info

If you encounter any issues with running the simulation, refer to the troubleshooting section at the end of this document.

The simulation library is modeled after OpenAI Gym and uses Pygame. Essentially, it provides a class SimpleBoatSim in `simple.py` that you use to simulate the environment. The way you use SimpleBoatSim is shown in `boat-test/main.py`; you need an outer loop that repeatedly calls `step(some action)` and `render()` (if you want to visualize the simulation). `step(action)` returns a state, a reward, whether the simulation terminated, and any other info.

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

- Refactor so that everything is in real world units (mostly done, need to integrate with all controllers)
- More realistic physics (eg add drag force, accelerations aren't applied instantaneously)
- More work on state estimation from noisy input, integrate with controllers
- Add boundaries to all controllers so that the boat doesn't go off very far from the screen
- Local map on main screen + minimap on top right showing global position so that the boat isn't extremely tiny on the main screen
- Change controllers so that they can also turn to a target heading

## Troubleshooting

**Pygame running slowly on macOS**

If this problem occurs, install Pygame version 2.0.0.dev6 as follows:

`pip install pygame==2.0.0.dev6`

Once it is installed, the simulation should run much more smoothly.
