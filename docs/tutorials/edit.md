# Editing an Onshape Assembly and Exporting to URDF

This tutorial demonstrates how to programmatically edit variables in an Onshape Variable Studio and then export the modified assembly to a URDF file using the `onshape-robotics-toolkit` library.

<img src="bike-header.gif" alt="Bike Header" style="width: 100%;">

---

## Prerequisites

Before you begin, make sure you have:

- **Installed the `onshape-robotics-toolkit` library**:
  ```bash
  pip install onshape-robotics-toolkit
  ```
- **API Keys**: Set up your Onshape API keys in a `.env` file as outlined in the [Getting Started](../getting-started.md) guide.
- **Access to the Onshape Document**: Use a CAD document with a Variable Studio. For this tutorial, we'll use the following example:
  <a href="https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/a86aaf34d2f4353288df8812" target="_blank">Example CAD Document</a>.

---

## Workflow

### Step 1: Set Up Logging and Initialize the Client

```python
from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.utilities import setup_default_logging

setup_default_logging(file_path="edit.log", console_level="INFO")

client = Client(env=".env")
```

The toolkit uses [loguru](https://github.com/Delgan/loguru) for logging. See also `setup_minimal_logging()`, `setup_quiet_logging()`, `setup_console_logging()`, and `setup_file_logging()` for finer control.

---

### Step 2: Load the Document and Read Variables

Use the document URL to create a `Document` object, then fetch the Variable Studio variables:

```python
from onshape_robotics_toolkit.models.document import Document

document = Document.from_url(
    url="https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/a86aaf34d2f4353288df8812"
)

elements = client.get_elements(document.did, document.wtype, document.wid)
variables = client.get_variables(document.did, document.wid, elements["variables"].id)
```

`client.get_elements()` returns a dict keyed by element type (e.g. `"variables"`, `"assembly"`).

---

### Step 3: Modify Variables and Push Changes

Edit variable expressions and write them back to Onshape:

```python
variables["wheelDiameter"].expression = "180 mm"
variables["wheelThickness"].expression = "71 mm"
variables["forkAngle"].expression = "20 deg"

client.set_variables(document.did, document.wid, elements["variables"].id, variables=variables)
```

Onshape regenerates the assembly with the new dimensions before the next API call.

---

### Step 4: Fetch the Updated Assembly

```python
assembly = client.get_assembly(document.did, document.wtype, document.wid, elements["assembly"].id)
```

---

### Step 5: Build the Robot Model

Parse the assembly and build the kinematic graph using the standard three-step pipeline:

```python
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.robot import Robot

cad = CAD.from_assembly(assembly, max_depth=2, client=client)
graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
robot = Robot.from_graph(kinematic_graph=graph, client=client, name="bike")
```

- `max_depth` controls how many levels of sub-assemblies are expanded (rigid vs. flexible).
- Pass `client` to `CAD.from_assembly` so rigid subassembly transforms and mass properties can be fetched when needed.

---

### Step 6: Export to URDF

```python
from onshape_robotics_toolkit.formats.urdf import URDFSerializer

URDFSerializer().save(
    robot=robot,
    file_path="output/edited_robot.urdf",
    download_assets=True,
    mesh_dir="output/meshes",
)
```

<img src="bike-urdf.gif" alt="Bike URDF" style="width: 100%;">

---

## Complete Script

```python
from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.formats.urdf import URDFSerializer
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.robot import Robot
from onshape_robotics_toolkit.utilities import setup_default_logging

DOCUMENT_URL = "https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/a86aaf34d2f4353288df8812"
MAX_DEPTH = 2

setup_default_logging(file_path="edit.log", console_level="INFO")

client = Client(env=".env")
document = Document.from_url(url=DOCUMENT_URL)

elements = client.get_elements(document.did, document.wtype, document.wid)
variables = client.get_variables(document.did, document.wid, elements["variables"].id)

variables["wheelDiameter"].expression = "180 mm"
variables["wheelThickness"].expression = "71 mm"
variables["forkAngle"].expression = "20 deg"

client.set_variables(document.did, document.wid, elements["variables"].id, variables=variables)

assembly = client.get_assembly(document.did, document.wtype, document.wid, elements["assembly"].id)

cad = CAD.from_assembly(assembly, max_depth=MAX_DEPTH, client=client)
graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
robot = Robot.from_graph(kinematic_graph=graph, client=client, name=f"edit_{MAX_DEPTH}")

URDFSerializer().save(
    robot=robot,
    file_path="output/edited_robot.urdf",
    download_assets=True,
    mesh_dir="output/meshes",
)
```

See [`examples/edit/main.py`](https://github.com/neurobionics/onshape-robotics-toolkit/blob/main/examples/edit/main.py) for the canonical version of this script.

---

## Result

After running the script, you'll find:

1. **`output/edited_robot.urdf`** — URDF file with the updated assembly dimensions
2. **`output/meshes/`** — STL mesh files referenced by the URDF
