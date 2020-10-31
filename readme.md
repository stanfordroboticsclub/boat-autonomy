# boat-autonomy

## Usage

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

The simulation library is modeled after OpenAI Gym and uses Pygame. Essentially, it provides a class SimpleBoatSim in `simple.py` that you use to simulate the environment. The way you use SimpleBoatSim is shown in `boat-test/main.py`; you need an outer loop that repeatedly calls `step(some action)` and `render()` (if you want to visualize the simulation). `step(action)` returns a state, a reward, whether the simulation terminated, and any other info.

### Controllers

| Controller          | ID                          | Description                                                                                                                            |
|---------------------|-----------------------------|----------------------------------------------------------------------------------------------------------------------------------------|
| KeyboardController  | `keyboard_controller`       | Move the boat with your the arrow keys of your keyboard.                                                                               |
| ComplementaryFilter | `complementary_filter_test` | Provides an experimental implementation of a complementary filter to estimate the state of the boat. For use with `sensor` state mode. |
| MinimalController   | `minimal_controller`        | Proof of concept controller that follows generated path. Works alright for small currents. For use with `ground_truth` state mode.     |


### State Representation

The `ground_truth` state is encoded as a list:

```
[boat x, boat y, boat speed, boat angle, boat angular velocity,
    [
        [obstacle 1 radius, obstacle 1 x, obstacle 1 y, obstacle 1 x-velocity, obstacle 1 y-velocity],
        [obstacle 2 radius, ...],
        ...
    ],
    [destination_x, destination_y]
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

- Refactor so that everything is in real world units (for example speed is now in pixels/sec)
- More realistic physics (eg add drag force)
- More work on state estimation from noisy input, integrate with minimal_controller
- Improve minimal_controller so that it can perform well in high currents; plan angular acceleration and linear acceleration 'together' rather than separately?
- Probably a lot more

## Troubleshooting

**Pygame running slowly on macOS**

If this problem occurs, install Pygame version 2.0.0.dev4 as follows:

`pip install pygame==2.0.0.dev6`

Once it is installed, the simulation should run much more smoothly.
