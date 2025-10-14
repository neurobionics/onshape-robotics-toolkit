# Exporting an Onshape Assembly to URDF

This tutorial demonstrates a streamlined workflow for converting an Onshape assembly to a URDF file using the `onshape-robotics-toolkit` library. Follow these steps to easily generate a URDF file and visualize your assembly's structure.

<img src="export-header.gif" alt="Export Header" style="width: 100%;">

---

## Prerequisites

Before you begin, ensure the following:

- **Install the library**: You have the `onshape-robotics-toolkit` library installed.
  ```bash
  pip install onshape-robotics-toolkit
  ```
- **API Keys**: Set up your Onshape API keys in a `.env` file. Refer to the [Getting Started](../getting-started.md) guide if needed.
- **Document URL**: Have the URL of the Onshape assembly you want to export. For this example, we'll use a quadruped robot assembly.

---

## Workflow: Onshape Assembly to JSON and Graph Visualization

### Step 1: Set Up Logging and Initialize the Client

Start by configuring the logger and initializing the Onshape API client:

```python
from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.utilities import setup_default_logging

# Configure logging: console at INFO + file at DEBUG
setup_default_logging(file_path="quadruped.log", console_level="INFO")

client = Client(env=".env")
```

The toolkit uses [loguru](https://github.com/Delgan/loguru) for logging. By default, loguru logs to stderr at DEBUG level. The `setup_default_logging()` helper provides a convenient way to configure both console and file logging with automatic rotation and retention. You can also use:

- `setup_minimal_logging()` for console-only output
- `setup_quiet_logging()` for file-only output
- `setup_console_logging()` and `setup_file_logging()` for fine-grained control

---

### Step 2: Load the Onshape Assembly

Use the `Robot` class to load the assembly directly from its Onshape document URL:

```python
from onshape_robotics_toolkit.robot import Robot

robot = Robot.from_url(
    name="quadruped",
    url="https://cad.onshape.com/documents/cf6b852d2c88d661ac2e17e8/w/c842455c29cc878dc48bdc68/e/b5e293d409dd0b88596181ef",
    client=client,
    max_depth=0,
    use_user_defined_root=False,
)
```

This will create a `Robot` object named "quadruped" from the specified Onshape document URL. The `max_depth` parameter controls the level of subassemblies to include, and `use_user_defined_root` specifies whether to use a user-defined root for the assembly.

---

### Step 3: Save the Assembly as JSON

Export the assembly data to a JSON file for easy analysis or integration with other tools:

```python
from onshape_robotics_toolkit.utilities.helpers import save_model_as_json

save_model_as_json(robot.assembly, "quadruped.json")
```

This will save the assembly details into a file named `quadruped.json` in the current working directory.

---

### Step 4: Visualize the Assembly Graph (Optional)

Generate and save a graphical representation of the assembly's structure:

```python
robot.show_graph(file_name="quadruped.png")
```

This will create a PNG file named `quadruped.png` showing the hierarchical structure of the assembly.

---

### Step 5: Save the Robot Object as a URDF File

If you plan to use the robot in a simulation environment, you can save the robot object as a URDF file:

```python
robot.save()
```

This saves the robot object to disk as a URDF file named `quadruped.robot`.

---

## Result

After running the script, you'll find the following files in your working directory:

1. **Assembly JSON File** (`quadruped.json`): Contains the complete assembly details.
2. **Assembly Graph** (`quadruped.png`): A visual representation of the assembly’s structure.
3. **Robot URDF File** (`quadruped.urdf`): A URDF file for simulation.

These files can be used for further analysis, simulation, or integration into other workflows.
