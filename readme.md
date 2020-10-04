# boat-autonomy


## Usage

- Download the folder named ‘boat’
- (Optionally install anaconda, create a venv, and activate it)
- `cd boat-simulation`
- `pip install -e .`
- `cd ../boat-test`
- `python main.py`

The simulation library is modelled after OpenAI Gym and uses Pygame. Essentially, it provides a class SimpleBoatSim in simple.py that you use to simulate the environment. The way you use SimpleBoatSim is shown in boat-test/main.py; you need an outer loop that repeatedly calls step(some action) and render() (if you want to visualize the simulation). step(action) returns a state, a reward, whether the simulation terminated, and any other info. The state is encoded as a list:

[boat x, boat y, boat speed, boat angle, boat angular velocity,
	[[obstacle 1 radius, obstacle 1 x, obstacle 1 y, obstacle 1 x velocity, obstacle 1 y
	velocity],
	[obstacle 2 radius, ...],
	…
	]
]

The elements of this list can be used by the robot to autonomously plan its path and avoid obstacles.

## Todos

- Detect collision of boat with obstacle and terminate simulation
- Add currents
- More diverse obstacles
- Change the state representation (for example don’t give exact coordinates but distances to nearby obstacles)
- Change what the actions do? (right now increase/decrease angular velocity + linear velocity)
- Make all the magic constants used actually align with physical units
- Add noise to state to simulate how estimates from sensors won’t be perfect
- Actual autonomy using the simulation
- Refactor code to make more clean, cut any inefficiencies, use pygame best practices
- Probably a lot more
