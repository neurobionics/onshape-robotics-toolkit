# Developer Documentation

This document provides an overview of how the onshape-robotics-toolkit library works internally and how different components interact with each other.

## Architecture Overview

The library is organized into several key components:

- **Client**: Handles all communication with the Onshape API
- **Robot**: Manages the robot model creation and URDF export
- **Assembly**: Represents the CAD assembly structure
- **Utilities**: Helper functions for various operations

## Workflow Overview

### 1. Authentication and Connection

The library first establishes a secure connection with Onshape:

- Users provide API credentials (access key and secret key)
- The Client component handles authentication and maintains the session
- All subsequent API requests use this authenticated session

### 2. Document Access

Once authenticated:

- The library can access Onshape documents using either URLs or direct document IDs
- Document metadata is retrieved to verify access and get necessary identifiers
- Assembly information is cached to minimize API calls

### 3. Assembly Processing

The assembly processing workflow:

1. Retrieves the full assembly structure from Onshape
2. Parses the hierarchical relationship between components
3. Identifies joints, mate connectors, and other relevant features
4. Creates an internal representation of the robot structure

### 4. URDF Generation

The URDF export process:

1. Analyzes the assembly structure
2. Maps Onshape constraints to URDF joints
3. Exports geometry in specified formats
4. Generates a complete URDF file with proper kinematics

## Sequence Diagram

The following sequence diagram illustrates the main interactions between components:

<pre class="mermaid">
sequenceDiagram
    participant User as User
    participant Client as onshape-robotics-toolkit.client
    participant Onshape as Onshape Server
    participant Robot as onshape-robotics-toolkit.robot

    User->>Client: Initialize Client with API Keys
    Client->>Onshape: Authenticate with Access Key & Secret Key
    Onshape-->>Client: Return Authentication Token

    User->>Client: Request Document Information (e.g., document URL)
    Client->>Onshape: Send GET request for Document Details
    Onshape-->>Client: Return Document Metadata (JSON)
    Client-->>User: Deliver Document Metadata

    User->>Client: Request CAD Assembly
    Client->>Onshape: Send GET request for Assembly Details
    Onshape-->>Client: Return Assembly Data (JSON)
    Client-->>User: Deliver Assembly Data

    User->>Robot: Initiate URDF Export Workflow
    Robot-->>Onshape: Parse Assembly Data for URDF
    Robot-->>User: Deliver Robot model (URDF)

    User->>User: Use URDF for Simulation or Control
</pre>

# Transforms Overview

## Creating URDF Robot Structure from Mates and Parts

This section explains how the Onshape Robotics Toolkit (ORT) converts Onshape assembly data into a proper URDF robot structure, focusing on the transformation matrices and coordinate system handling.

## Discrepancy in the Order of Mate Entities

### Onshape User Interface vs. Assembly Data

1. **User Selection Order**: In Onshape, when creating mates, users select the child part (the one that needs to move to the parent's location) first, then the parent part.

2. **Assembly Data Storage**: The assembly JSON document saves this as `[PARENT, CHILD]` under `matedEntities` for each mate feature. This means the underlying data considers the inverse of what the user intended.

## How ORT Parses Mate Features

### Entity Order Convention

ORT establishes a consistent ordering convention:

```python
CHILD = 0
PARENT = 1
```

This convention is used throughout the transformation pipeline to ensure consistent coordinate system handling.

### Transformation Pipeline

Once all mates are collected from the assembly, transformations are applied in two key methods:

- `get_robot_link()` - Handles link coordinate transformations
- `get_robot_joint()` - Handles joint coordinate transformations

## Transforming Links

### Root Node Processing

1. **Root Link Initialization**: The root node is processed first without any mate reference. The center of mass (COM) of this root part becomes the world origin.

```python
if mate is None:
    _link_to_stl_tf[:3, 3] = np.array(part.MassProperty.center_of_mass).reshape(3)
```

### Child Link Processing

2. **Child Links with Mates**: For subsequent parts/links connected via mates, we extract the `part_to_mate_tf` transformation, which represents the transformation from the part's coordinate system to the mate coordinate system.

```python
else:
    _link_to_stl_tf = mate.matedEntities[CHILD].matedCS.part_to_mate_tf
```

For rigid assemblies, an additional parent coordinate system transformation is applied:

```python
elif mate.matedEntities[CHILD].parentCS:
    _link_to_stl_tf = mate.matedEntities[CHILD].parentCS.part_tf @ mate.matedEntities[CHILD].matedCS.part_to_mate_tf
```

### STL Mesh Transformation

3. **Mesh Coordinate Adjustment**: The transformation matrix is inverted to convert from link coordinates to STL coordinates, ensuring proper COM and inertia representation.

```python
_stl_to_link_tf = np.matrix(np.linalg.inv(_link_to_stl_tf))
```

4. **Link Origin Convention**: The origin of any link is always a zero vector since both the STL mesh and joint coordinates are transformed to maintain consistency.

## Transforming Joints

### Joint Coordinate System

Joint transformations use the parent link's transformation as a reference:

```python
# For regular assemblies
parent_to_mate_tf = mate.matedEntities[PARENT].matedCS.part_to_mate_tf

# For rigid assemblies
parent_to_mate_tf = (
    mate.matedEntities[PARENT].parentCS.part_tf @
    mate.matedEntities[PARENT].matedCS.part_to_mate_tf
)
```

### Final Joint Transformation

The final joint transformation combines the parent link transformation with the mate transformation:

```python
stl_to_mate_tf = stl_to_parent_tf @ parent_to_mate_tf
origin = Origin.from_matrix(stl_to_mate_tf)
```

## Key Transformation Matrices

| Matrix            | Description                                     | Usage                                                                |
| ----------------- | ----------------------------------------------- | -------------------------------------------------------------------- |
| `part_to_mate_tf` | Part coordinate system → Mate coordinate system | Fundamental transformation from part origin to mate coordinate frame |
| `_link_to_stl_tf` | Link coordinate system → STL coordinate system  | Used to position the part geometry correctly                         |
| `_stl_to_link_tf` | STL coordinate system → Link coordinate system  | Inverse transformation for mesh positioning                          |
| `stl_to_mate_tf`  | STL coordinate system → Mate coordinate system  | Final joint origin transformation                                    |

## Coordinate System Assumptions

### Part Coordinate Systems

- **Part Origin**: Each part has its own local coordinate system as defined in Onshape
- **Center of Mass**: Used as reference for root link positioning
- **Mate Coordinate Systems**: Defined by mate features and represent joint locations

### URDF Coordinate Systems

- **Link Origins**: Always at zero vector after transformation
- **Joint Origins**: Positioned at mate coordinate system locations
- **Mesh Positioning**: STL files are transformed to align with link coordinate systems

## Example Workflow

1. **Assembly Graph Traversal**: Process assembly in topological order starting from root
2. **Root Link Creation**: `get_robot_link(name=root_node, mate=None)` - uses COM as origin
3. **Child Link Processing**: For each edge `(parent, child)` in assembly graph:
   - Extract mate connecting parent and child
   - Call `get_robot_link(name=child, mate=mate)` - uses mate transformation
   - Call `get_robot_joint(parent, child, mate, parent_tf)` - creates joint
4. **Transformation Propagation**: Each link's transformation is stored for use by its children

This systematic approach ensures that the resulting URDF maintains proper kinematic relationships while respecting Onshape's coordinate system conventions.

## Key Classes and Their Roles

### Client Class

- Manages API authentication
- Handles all HTTP requests to Onshape
- Implements rate limiting and error handling
- Caches responses when appropriate

### Robot Class

- Represents the complete robot model
- Manages the conversion from CAD assembly to URDF
- Handles coordinate transformations
- Provides visualization utilities

### Assembly Class

- Represents the CAD assembly structure
- Maintains parent-child relationships
- Tracks mate connections and constraints
- Manages geometric transformations
