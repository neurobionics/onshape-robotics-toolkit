# Simulation Example

This example demonstrates how to use the onshape-robotics-toolkit with MuJoCo for physics simulation and Optuna for design optimization of a ballbot robot.

## Additional Dependencies

This example requires additional dependencies that are not part of the core toolkit. Install them using:

```bash
pip install onshape-robotics-toolkit[simulation]
```

Or if you're using uv:

```bash
uv pip install onshape-robotics-toolkit[simulation]
```

This will install:

- `mujoco` - Physics simulation engine
- `optuna` - Hyperparameter optimization framework
- `plotly` - Interactive visualization library
- `usd-core` - Universal Scene Description for 3D scene export

## Running the Example

After installing the simulation dependencies, you can run the example:

```bash
python examples/simulation/main.py
```

The script will prompt you for a run name and then perform:

1. Design optimization (finding optimal physical parameters)
2. Control optimization (tuning PID controllers)
3. Export simulation results as USD files and visualization plots
