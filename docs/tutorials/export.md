# Exporting an Onshape Assembly to URDF

This tutorial demonstrates the workflow for converting an Onshape assembly to a URDF file using the `onshape-robotics-toolkit` library.

<img src="export-header.gif" alt="Export Header" style="width: 100%;">

---

## Prerequisites

Before you begin, ensure the following:

- **Install the library**: You have the `onshape-robotics-toolkit` library installed.
  ```bash
  pip install onshape-robotics-toolkit
  ```
- **API Keys**: Set up your Onshape API keys in a `.env` file. Refer to the [Getting Started](../getting-started.md) guide if needed.
- **Document URL**: Have the URL of the Onshape assembly you want to export.

---

## Workflow: CAD → KinematicGraph → Robot → URDF

The pipeline follows three stages:

1. **`CAD`** — fetch and parse the Onshape assembly
2. **`KinematicGraph`** — build the robot topology
3. **`Robot`** — construct the robot model and export it

### Step 1: Set Up Logging and Initialize the Client

```python
from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.utilities import setup_default_logging

setup_default_logging(file_path="quadruped.log", console_level="INFO")

client = Client(env=".env")
```

The toolkit uses [loguru](https://github.com/Delgan/loguru) for logging. The `setup_default_logging()` helper configures both console and file logging. Alternatives:

- `setup_minimal_logging()` — console only
- `setup_quiet_logging()` — file only

---

### Step 2: Load the Onshape Assembly

```python
from onshape_robotics_toolkit.parse import CAD

cad = CAD.from_url(
    "https://cad.onshape.com/documents/cf6b852d2c88d661ac2e17e8/w/c842455c29cc878dc48bdc68/e/b5e293d409dd0b88596181ef",
    client=client,
    max_depth=0,
)
```

`max_depth` controls how many levels of sub-assemblies to include (`0` = top-level only, `1` = one level deep, etc.).

---

### Step 3: Build the Kinematic Graph

```python
from onshape_robotics_toolkit.graph import KinematicGraph

graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
```

Set `use_user_defined_root=False` if the assembly has no user-defined root part; the library will pick one automatically.

---

### Step 4: Construct the Robot Model

```python
from onshape_robotics_toolkit.robot import Robot

robot = Robot.from_graph(kinematic_graph=graph, client=client, name="quadruped")
```

---

### Step 5: Export to URDF

```python
from onshape_robotics_toolkit.formats.urdf import URDFSerializer

serializer = URDFSerializer()
serializer.save(
    robot,
    "output/quadruped.urdf",
    download_assets=True,
    mesh_dir="output/meshes",
)
```

`download_assets=True` fetches the STL mesh files from Onshape and saves them alongside the URDF.

---

## Complete Script

```python
from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.formats.urdf import URDFSerializer
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.robot import Robot
from onshape_robotics_toolkit.utilities import setup_default_logging

DOCUMENT_URL = "https://cad.onshape.com/documents/cf6b852d2c88d661ac2e17e8/w/c842455c29cc878dc48bdc68/e/b5e293d409dd0b88596181ef"

setup_default_logging(file_path="quadruped.log", console_level="INFO")

client = Client(env=".env")
cad = CAD.from_url(DOCUMENT_URL, client=client, max_depth=0)
graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
robot = Robot.from_graph(kinematic_graph=graph, client=client, name="quadruped")

URDFSerializer().save(robot, "output/quadruped.urdf", download_assets=True, mesh_dir="output/meshes")
```

See [`examples/export/main.py`](https://github.com/neurobionics/onshape-robotics-toolkit/blob/main/examples/export/main.py) for an extended version that also reads from an `ORTConfig` YAML file.

---

## Result

After running the script, you'll find:

1. **`output/quadruped.urdf`** — URDF file ready for simulation
2. **`output/meshes/`** — STL mesh files referenced by the URDF
