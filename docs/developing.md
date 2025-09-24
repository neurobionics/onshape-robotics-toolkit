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

# Assembly Pattern Support

## Overview

Assembly patterns in Onshape allow users to create multiple instances of parts or subassemblies in regular arrangements (linear, circular, etc.). The onshape-robotics-toolkit supports these patterns by automatically expanding mate features to connect all pattern instances to their target parts.

## Pattern Processing Workflow

### 1. Pattern Detection and Mapping

The toolkit first identifies:

- **Seed entities**: The original parts/subassemblies that are patterned
- **Pattern instances**: All the generated copies from the pattern
- **Seed mates**: Mate features that connect seed entities to other parts

### 2. Mate Expansion Strategy

For each seed mate that involves a patterned entity:

1. Create a deep copy of the original mate for each pattern instance
2. Update the mate to reference the pattern instance instead of the seed
3. Calculate the new mate coordinate system for the pattern instance
4. Preserve the relative relationship between the pattern instance and the connected part

## Transform Calculations for Pattern Mates

### The Challenge

When parts are patterned, their mate connections need to be updated to account for the new positions of pattern instances. The key challenge is maintaining the correct relative positioning between patterned parts and their connected (non-patterned) parts, regardless of where the assembly is positioned in world space.

### Coordinate System Flow

The transformation pipeline for pattern mate expansion follows this sequence:

```python
# 1. Get world transforms for seed, pattern, and connected entities
seed_entity_occurrence_tf = occurrences.get(seed_occurrence_id).transform
pattern_entity_occurrence_tf = occurrences.get(pattern_instance_id).transform
other_entity_occurrence_tf = occurrences.get(other_entity_occurrence_id).transform

# 2. Calculate relative transform from seed to pattern position
seed_to_pattern_relative_tf = pattern_entity_occurrence_tf @ np.linalg.inv(seed_entity_occurrence_tf)

# 3. Transform through coordinate spaces: local → world → pattern_relative → local
other_entity_mate_world_tf = other_entity_occurrence_tf @ original_mate_cs.part_to_mate_tf
pattern_transformed_world_tf = seed_to_pattern_relative_tf @ other_entity_mate_world_tf
final_mate_cs = np.linalg.inv(other_entity_occurrence_tf) @ pattern_transformed_world_tf
```

### Key Transform Matrices

| Matrix                         | Description                                   | Purpose                                 |
| ------------------------------ | --------------------------------------------- | --------------------------------------- |
| `seed_entity_occurrence_tf`    | Seed entity's world transform                 | Establishes original world position     |
| `pattern_entity_occurrence_tf` | Pattern instance's world transform            | New world position for pattern instance |
| `other_entity_occurrence_tf`   | Connected entity's world transform            | Reference frame for mate coordinates    |
| `seed_to_pattern_relative_tf`  | Relative transform: `M_pattern * M_seed^(-1)` | Captures the pattern transformation     |

### Relative Pose Calculation

The relative transformation between seed and pattern positions uses the formula:

```
M_relative = M_pattern * M_seed^(-1)
```

Where:

- `M_seed` is the seed entity's world transformation matrix
- `M_pattern` is the pattern instance's world transformation matrix
- `M_relative` captures how to transform from seed position to pattern position

### Coordinate System Preservation

The critical insight is that mate coordinates must remain **relative to the connected entity**, not world origin. The transformation pipeline ensures this by:

1. **Local → World**: Transform original mate coordinates to world space using the connected entity's world transform
2. **Apply Pattern Transform**: Apply the seed-to-pattern relative transformation in world coordinates
3. **World → Local**: Transform back to the connected entity's local coordinate system

This approach ensures that:

- Joint locations remain consistent regardless of assembly world position
- Pattern transformations are applied correctly in world space
- Final mate coordinates are properly expressed relative to the connected part

### Floating Point Precision Handling

Pattern transformations involve matrix inversion and multiplication chains that can introduce floating point precision errors. The toolkit applies numerical tolerance cleanup:

```python
tolerance = 1e-10
# Clean up intermediate results
seed_to_pattern_relative_tf[np.abs(seed_to_pattern_relative_tf) < tolerance] = 0.0
# Clean up final transformation
final_mate_cs[np.abs(final_mate_cs) < tolerance] = 0.0
```

This ensures that near-zero values (from floating point artifacts) are set to exactly zero, preventing small joint location errors in the generated URDF.

# Hierarchical Rigid Subassembly Support

## Overview

The onshape-robotics-toolkit supports complex scenarios where parts are buried multiple levels deep within rigid subassemblies. This is a common CAD modeling pattern where parts are organized within nested sub-assemblies that are then marked as rigid for kinematic purposes.

## The Challenge

Consider this assembly hierarchy:

```
Root Assembly
├── Mobile Part A
├── Rigid Subassembly X
│   ├── Sub-Assembly Level 1
│   │   ├── Sub-Assembly Level 2
│   │   │   └── Target Part B
│   │   └── Other Parts...
│   └── Other Sub-Assemblies...
└── Mobile Part C (mated to Target Part B)
```

When a mate connects `Mobile Part C` to `Target Part B`, the toolkit needs to calculate the proper transform chain from the `Rigid Subassembly X` root down to `Target Part B` through all intermediate sub-assembly levels.

## Coordinate System Considerations

### Global vs. Local Coordinate Systems

The key insight is distinguishing between two types of occurrence transforms:

1. **Global Occurrences**: Transforms in the main assembly's world coordinate system
2. **Rigid Subassembly Occurrences**: Transforms relative to the rigid subassembly's root coordinate system

For proper URDF generation, we need transforms **relative to the rigid subassembly root**, not global world coordinates.

### Transform Chain Calculation

The hierarchical transform calculation follows this pattern:

```python
# For occurrences_list = [rigid_sub, level1, level2, target_part]
parent_tf = np.eye(4)

# Build hierarchical keys and multiply transforms
for i in range(len(occurrences_list) - 1):
    # Key construction excludes rigid subassembly name
    hierarchical_key = SUBASSEMBLY_JOINER.join(occurrences_list[1:i+2])
    # Keys: "level1", "level1_JOINER_level2"

    occurrence_tf = rigid_subassembly_occurrences[hierarchical_key].transform
    parent_tf = parent_tf @ occurrence_tf  # Sequential multiplication
```

## Implementation Details

### Hierarchical Key Construction

The rigid subassembly occurrence map stores keys **relative to the rigid subassembly**:

| Assembly Structure                  | Global Key                                          | Rigid Subassembly Key              |
| ----------------------------------- | --------------------------------------------------- | ---------------------------------- |
| `[rigid_sub, level1]`               | `rigid_sub_JOINER_level1`                           | `level1`                           |
| `[rigid_sub, level1, level2]`       | `rigid_sub_JOINER_level1_JOINER_level2`             | `level1_JOINER_level2`             |
| `[rigid_sub, level1, level2, part]` | `rigid_sub_JOINER_level1_JOINER_level2_JOINER_part` | `level1_JOINER_level2_JOINER_part` |

### Transform Chain Building

```python
def build_hierarchical_transform_for_rigid_subassembly(
    occurrences_list: list[str],
    rigid_subassembly_occurrence_map: dict[str, dict[str, Occurrence]],
) -> np.matrix:
    """
    Build hierarchical transform chain for a part buried within a rigid subassembly.

    Returns: Transform matrix from rigid subassembly root to part coordinate system
    """
```

The function:

1. **Validates input**: Ensures the rigid subassembly exists in the occurrence map
2. **Iterates through levels**: Processes each sub-assembly level (excluding final part)
3. **Builds cumulative transform**: Multiplies transforms sequentially
4. **Returns final matrix**: Complete transform from rigid subassembly root to part

### Usage in Mate Processing

The hierarchical transform is applied in multiple contexts:

#### Regular Mate Processing

```python
if child_occurrences[0] in rigid_subassemblies:
    parent_tf = build_hierarchical_transform_for_rigid_subassembly(
        child_occurrences, rigid_subassembly_occurrence_map
    )
    child_parentCS = MatedCS.from_tf(parent_tf)
    feature.featureData.matedEntities[CHILD].parentCS = child_parentCS
```

#### Pattern Mate Processing

The same logic applies to pattern-expanded mates, ensuring consistent transform handling for both seed entities and pattern instances.

#### Parent and Child Entities

Both parent and child entities in mates receive hierarchical transform support when they reside within rigid subassemblies.

## Depth Agnostic Design

The implementation automatically handles any nesting depth:

- **2 levels**: `[rigid_sub, part]` - Single iteration, direct transform
- **3 levels**: `[rigid_sub, level1, part]` - One intermediate level
- **N levels**: `[rigid_sub, level1, ..., levelN-1, part]` - N-2 intermediate levels

The algorithm scales without modification as `range(len(occurrences_list) - 1)` automatically adapts to the hierarchy depth.

## Integration with URDF Generation

The calculated `parentCS` transforms integrate with the existing URDF generation pipeline:

### Link Transform Calculation

```python
if mate.matedEntities[CHILD].parentCS:
    _link_to_stl_tf = mate.matedEntities[CHILD].parentCS.part_tf @ mate.matedEntities[CHILD].matedCS.part_to_mate_tf
```

### Joint Transform Calculation

```python
parent_to_mate_tf = (
    mate.matedEntities[PARENT].parentCS.part_tf @
    mate.matedEntities[PARENT].matedCS.part_to_mate_tf
)
```

This ensures that parts buried within complex rigid subassembly hierarchies are positioned correctly in the final URDF, maintaining proper kinematic relationships regardless of their nesting depth within the CAD assembly structure.

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
