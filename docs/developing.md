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

# Disconnected Graph Resolution

## Overview

When processing assemblies at shallow depths (e.g., `max_depth=1`), the toolkit may encounter scenarios where mate features reference individual parts within rigid subassemblies instead of the rigid subassembly roots. This creates disconnected subgraphs that result in incomplete robot models with missing parts and joints.

## The Problem

### Root Cause

At `max_depth=1`, rigid subassemblies are downloaded as single parts to optimize performance. However, mate features still contain references to the individual parts that existed before the subassembly was treated as rigid. This mismatch causes:

1. **Graph Disconnection**: Mate keys reference non-existent individual parts instead of rigid subassembly roots
2. **Component Removal**: The graph processing routine removes disconnected components
3. **Incomplete Models**: Final URDF lacks parts and joints that should be included

### Example Scenario

```
Assembly Structure (max_depth=1):
├── Motor (individual part)
├── Wheel Subassembly (rigid, downloaded as single part)
│   ├── Hub (not individually accessible)
│   └── Rollers (not individually accessible)
└── Mate: Motor ↔ Hub (references non-accessible Hub part)
```

The mate references `Hub` which doesn't exist at `max_depth=1`, causing the connection to fail.

## The Solution: Instance Proxy Mapping

### Proxy Map Generation

The `get_instances()` function creates an `instance_proxy_map` that maps individual part instance keys to their rigid subassembly container names:

```python
instance_proxy_map = {
    "wheel_subassembly_JOINER_hub": "wheel_subassembly",
    "wheel_subassembly_JOINER_roller_1": "wheel_subassembly",
    # ... other mappings
}
```

### Mate Key Resolution

#### Core Functions

**`get_proxy_occurrence_name()`**: Maps occurrence paths to their proxy names using the instance proxy map.

```python
def get_proxy_occurrence_name(
    occurrences: list[str],
    instance_proxy_map: dict[str, str],
    subassembly_prefix: Optional[str] = None
) -> str:
    # Build full occurrence key
    prefix = f"{subassembly_prefix}{SUBASSEMBLY_JOINER}" if subassembly_prefix else ""
    occurrence_key = f"{prefix}{SUBASSEMBLY_JOINER.join(occurrences)}"

    # Use proxy mapping if available, otherwise use original path
    return instance_proxy_map.get(occurrence_key, SUBASSEMBLY_JOINER.join(occurrences))
```

**`join_mate_occurrences_with_proxy()`**: Creates mate keys using proxy-mapped occurrence names.

```python
def join_mate_occurrences_with_proxy(
    parent_occurrences: list[str],
    child_occurrences: list[str],
    instance_proxy_map: dict[str, str],
    subassembly_prefix: Optional[str] = None
) -> str:
    parent_name = get_proxy_occurrence_name(parent_occurrences, instance_proxy_map, subassembly_prefix)
    child_name = get_proxy_occurrence_name(child_occurrences, instance_proxy_map, subassembly_prefix)
    return f"{parent_name}{MATE_JOINER}{child_name}"
```

### Integration Points

#### Regular Mate Processing

```python
# In process_features_async()
mate_key = join_mate_occurrences_with_proxy(
    parent_occurrences,
    child_occurrences,
    instance_proxy_map,
    subassembly_prefix
)
```

#### Pattern Mate Processing

```python
# In process_patterns_async()
for pattern_instance in pattern_instances:
    pattern_mate_key = join_mate_occurrences_with_proxy(
        pattern_parent_occurrences,
        pattern_child_occurrences,
        instance_proxy_map,
        subassembly_prefix
    )
```

## Unique Mate Name Resolution

### The Challenge

Pattern expansion can create multiple mates with identical names (e.g., `Revolute_1` appearing twice), making the URDF invalid since joint names must be unique.

### Solution: Iterative Conflict Resolution

The `ensure_unique_mate_names()` function implements an iterative approach to resolve naming conflicts:

```python
def ensure_unique_mate_names(mates: dict[str, MateFeatureData]) -> None:
    max_iterations = 10
    iteration = 0

    while iteration < max_iterations:
        # Count occurrences of each mate name
        name_counts = Counter(mate.featureData.name for mate in mates.values())

        # Find and resolve conflicts
        for name, count in name_counts.items():
            if count > 1:
                # Generate unique suffixes for duplicates
                # Ensure new names don't conflict with existing names
```

#### Key Features

1. **Iterative Resolution**: Handles complex chains of naming conflicts
2. **Conflict Avoidance**: Ensures new names don't create additional conflicts
3. **Deterministic Ordering**: Processes mates in consistent order for reproducible results
4. **Bounded Execution**: Limits iterations to prevent infinite loops

## Workflow Integration

### Processing Pipeline

1. **Instance Mapping**: `get_instances()` creates `instance_proxy_map`
2. **Feature Processing**: Pass `instance_proxy_map` through processing pipeline
3. **Mate Key Generation**: Use proxy-aware functions for all mate key creation
4. **Name Uniqueness**: Apply `ensure_unique_mate_names()` before URDF generation
5. **Graph Construction**: Connected graph with proper rigid subassembly references

### Result Validation

The solution ensures:

- **Connected Graphs**: All parts properly linked through proxy-mapped mate keys
- **Unique Joint Names**: URDF compliance with distinct joint identifiers
- **Consistent Transforms**: Proper coordinate system handling for rigid subassemblies
- **Pattern Support**: Correct expansion of mates for patterned components

This comprehensive approach resolves disconnected graph issues while maintaining the performance benefits of shallow assembly processing depths.

# Mixed Flexible-Rigid Assembly Support

## Overview

The onshape-robotics-toolkit supports complex scenarios that occur at intermediate `max_depth` values (typically `max_depth=1`) where assemblies contain a mixture of flexible components and rigid subassemblies. This creates a challenging coordinate system scenario where mates reference parts using different naming conventions and hierarchical structures.

## The Challenge: Name Mapping Mismatch

### Problem Description

At `max_depth=1`, the toolkit encounters a specific challenge:

1. **Flexible assemblies** remain expanded and their individual parts are accessible
2. **Rigid subassemblies** are collapsed and treated as single entities
3. **Mate features** reference individual parts using sanitized names (e.g., `double-wheel_1`)
4. **Rigid subassembly keys** use prefixed names (e.g., `wheel_1_78C6_double-wheel_1`)

This mismatch prevents the hierarchical transform logic from correctly identifying when rigid subassembly transforms need to be applied.

### Example Scenario

```
Assembly at max_depth=1:
├── Part_1_1 (individual part, flexible)
├── wheel_1 (flexible assembly)
│   ├── double-wheel_1 (rigid subassembly → wheel_1_78C6_double-wheel_1)
│   │   ├── Part_4_1 (internal part)
│   │   ├── single-wheel_1 (rigid subassembly → wheel_1_78C6_double-wheel_1_78C6_single-wheel_1)
│   │   └── single-wheel_2 (rigid subassembly → wheel_1_78C6_double-wheel_1_78C6_single-wheel_2)
│   └── Part_3_1 (individual part, flexible)
└── Mate: Part_1_1 ↔ ['wheel_1', 'double-wheel_1', 'Part_4_1']
```

The mate references `double-wheel_1` but the rigid subassembly map contains `wheel_1_78C6_double-wheel_1`, causing the hierarchical transform detection to fail.

## The Solution: Sanitized-to-Prefixed Name Mapping

### Dynamic Name Mapping Creation

The mate processing logic now creates a dynamic mapping from sanitized names to prefixed names:

```python
# Create a mapping from sanitized names to prefixed names for rigid subassemblies
rigid_subassembly_sanitized_to_prefixed = {}
for prefixed_key in rigid_subassemblies.keys():
    # Extract the sanitized name from the prefixed key
    sanitized_name = prefixed_key.split(SUBASSEMBLY_JOINER)[-1]
    rigid_subassembly_sanitized_to_prefixed[sanitized_name] = prefixed_key
```

Example mapping:

```python
{
    'double-wheel_1': 'wheel_1_78C6_double-wheel_1',
    'single-wheel_1': 'wheel_1_78C6_double-wheel_1_78C6_single-wheel_1',
    'single-wheel_2': 'wheel_1_78C6_double-wheel_1_78C6_single-wheel_2'
}
```

### Updated Detection Logic

The rigid subassembly detection logic now uses the sanitized names for checking:

```python
# Before: Failed to detect rigid subassemblies
if parent_occurrences[0] in rigid_subassemblies:
    # This would fail because 'double-wheel_1' != 'wheel_1_78C6_double-wheel_1'

# After: Correctly detects rigid subassemblies
if parent_occurrences[0] in rigid_subassembly_sanitized_to_prefixed:
    # This succeeds because 'double-wheel_1' is in the mapping
    prefixed_parent_name = rigid_subassembly_sanitized_to_prefixed[parent_occurrences[0]]
```

### Hierarchical Transform Application

When hierarchical transforms are needed, the system:

1. **Maps sanitized to prefixed names** for rigid subassembly lookups
2. **Updates occurrence lists** to use prefixed names in transform functions
3. **Sets parentCS transforms** using the correct coordinate systems
4. **Updates mate occurrence paths** to point to rigid subassembly components

```python
# Apply hierarchical transform using prefixed names
prefixed_parent_occurrences = [prefixed_parent_name] + parent_occurrences[1:]
parent_tf = build_hierarchical_transform_for_rigid_subassembly(
    prefixed_parent_occurrences, rigid_subassembly_occurrence_map
)
parent_parentCS = MatedCS.from_tf(parent_tf)
feature.featureData.matedEntities[PARENT].parentCS = parent_parentCS

# Use prefixed name for mate connection graph
parent_occurrences = [prefixed_parent_name]
```

## Enhanced Function Support

### Updated build_hierarchical_transform_with_flexible_parents_v2()

A new version of the flexible parents transform function accepts the sanitized-to-prefixed mapping:

```python
def build_hierarchical_transform_with_flexible_parents_v2(
    occurrences_list: list[str],
    rigid_subassembly_occurrence_map: dict[str, dict[str, Occurrence]],
    flexible_occurrences: dict[str, Occurrence],
    rigid_subassembly_sanitized_to_prefixed: dict[str, str],
    subassembly_prefix: Optional[str] = None,
) -> np.matrix:
```

Key improvements:

- Uses sanitized-to-prefixed mapping for rigid subassembly detection
- Updates occurrence lists with prefixed names before calling hierarchical transform functions
- Maintains proper coordinate system relationships

### Coordinate System Resolution

The fix ensures proper coordinate system resolution by:

1. **Detecting mixed scenarios**: Identifying when mates span flexible-rigid boundaries
2. **Applying correct transforms**: Using hierarchical transforms for rigid subassembly parts
3. **Maintaining graph connectivity**: Ensuring mate keys reference correct rigid subassembly components
4. **Preserving coordinate accuracy**: Applying parentCS transforms to fix coordinate offsets

## Impact on URDF Generation

### Transform Correction

The fix resolves coordinate system issues that previously caused:

- **Incorrect joint origins**: 20mm offsets due to missing hierarchical transforms
- **Disconnected graphs**: Parts not properly connected due to name mismatches
- **Missing components**: Rigid subassembly parts excluded from final URDF

### Example Results

**Before (Incorrect coordinates):**

```xml
<joint name="Revolute_1_2" type="revolute">
  <origin xyz="0 0.036 -0.024" rpy="0 0 0"/>  <!-- Wrong: -24mm offset -->
</joint>
```

**After (Correct coordinates):**

```xml
<joint name="Revolute_1_2" type="revolute">
  <origin xyz="0 0.036 0" rpy="0 0 0"/>  <!-- Correct: proper coordinate -->
</joint>
```

## Debugging and Validation

### Debug Logging

The implementation includes comprehensive debug logging:

```python
LOGGER.debug(f"Rigid subassembly name mapping: {rigid_subassembly_sanitized_to_prefixed}")
LOGGER.debug(f"Parent[0] in rigid_subassemblies: {parent_occurrences[0] in rigid_subassembly_sanitized_to_prefixed}")
LOGGER.debug(f"Using parentCS for joint from {parent} to {child}")
```

### Validation Criteria

The fix ensures:

- **Hierarchical transforms applied**: parentCS transforms are set when needed
- **Correct coordinate systems**: Joint origins use proper coordinate calculations
- **Connected component graphs**: All parts properly linked in assembly graph
- **Unique naming**: Rigid subassembly components have distinct prefixed names

## Integration with Existing Systems

### Backward Compatibility

The mixed flexible-rigid support integrates seamlessly with existing systems:

- **Pattern processing**: Works with both regular and pattern-expanded mates
- **Rigid subassembly hierarchies**: Compatible with existing hierarchical transform logic
- **Graph processing**: Maintains existing graph construction and validation
- **URDF generation**: Uses established coordinate transformation pipeline

### Performance Considerations

The name mapping approach:

- **Minimal overhead**: O(n) mapping creation where n = number of rigid subassemblies
- **Memory efficient**: Small dictionaries with string mappings
- **Processing optimized**: Single pass through mate features with cached mappings
- **Deterministic results**: Consistent mapping ensures reproducible URDF output

This comprehensive solution enables accurate URDF generation for complex mixed flexible-rigid assemblies at intermediate max_depth values while maintaining compatibility with all existing assembly processing features.

# CAD Class and Assembly Data Structure

## Overview

The `CAD` class provides a modern, hierarchical representation of Onshape assemblies using `PathKey`-based indexing. This replaced the older string-based key system to enable consistent data access across all registries.

## Key Design Principles

### 1. Hierarchical Data Preservation

The assembly data structure preserves Onshape's hierarchy instead of flattening it:

- **Root Assembly**: Contains all instances (flat registry for lookup)
- **Subassemblies**: Each has its own `AssemblyData` with mates and patterns
- **Parts**: Keyed by `PathKey`, includes both regular parts and rigid subassemblies

```python
cad.root_assembly.instances  # Flat registry: all instances
cad.sub_assemblies           # Hierarchical: dict[PathKey, AssemblyData]
cad.parts                    # Flat registry: dict[PathKey, Part]
```

### 2. PathKey-Based Indexing

All data structures use `PathKey` for consistent indexing:

```python
# PathKey examples
root_part = PathKey(('M0cLO6yVimMv6KhRM',))
nested_part = PathKey(('McQ65EsxX+4zImFWp', 'MUgiNm7M17UkTi3/g'))

# Access patterns
instance = cad.root_assembly.instances.parts[key]
part_data = cad.parts[key]
occurrence = cad.root_assembly.occurrences.occurrences[key]
```

### 3. Rigid Assembly Handling

Rigid assemblies (those beyond `max_depth`) are treated as parts:

- **Marking**: `AssemblyInstance.isRigid` flag set during population
- **Storage**: Rigid assemblies added to `cad.parts` as `Part` objects with `isRigidAssembly=True`
- **Detection**: Use `InstanceRegistry.is_rigid_assembly(key)` which checks the `isRigid` flag

```python
# Rigid assemblies are marked during population
if key.depth > self.max_depth:
    instance.isRigid = True

# Rigid assemblies become Part objects
if instance.isRigid:
    cad.parts[key] = Part(..., isRigidAssembly=True)
```

## Data Flow in from_assembly()

The `CAD.from_assembly()` method follows this sequence:

1. **Create root assembly data** - Populate from `rootAssembly` JSON
2. **Populate subassemblies** - Process each subassembly:
   - Add instances to both root (flat) and subassembly (hierarchical) registries
   - Mark rigid assemblies based on depth
   - Store mates and patterns only in subassembly registries
3. **Populate parts** - Create Part objects:
   - Regular parts from `assembly.parts` JSON
   - Rigid subassemblies as Part objects with `isRigidAssembly=True`

## Instance Storage Strategy

Instances are stored in two places to balance lookup efficiency and hierarchy preservation:

| Registry                        | Contains                           | Purpose                          |
| ------------------------------- | ---------------------------------- | -------------------------------- |
| `root_assembly.instances`       | All instances (parts + assemblies) | Fast flat lookup by PathKey      |
| `sub_assemblies[key].instances` | Instances within that subassembly  | Preserves hierarchical structure |

Mates and patterns remain exclusive to subassemblies (not duplicated in root).

## Rigid Assembly Detection

Always use the `isRigid` flag rather than depth comparison:

```python
# ✅ Correct: Uses isRigid flag
if cad.root_assembly.instances.is_rigid_assembly(key):
    # Handle rigid assembly

# ❌ Avoid: Direct depth comparison (use during initial population only)
if key.depth > max_depth:
    # Only appropriate when SETTING the isRigid flag
```

The `isRigid` flag is the source of truth after assembly population is complete.

## Integration Notes

### Fetching Flexible Subassemblies

The `fetch_subassemblies()` method uses `is_flexible_assembly()` to identify which assemblies need API fetching:

```python
flexible_assemblies = [
    (key, instance)
    for key, instance in cad.root_assembly.instances.assemblies.items()
    if cad.root_assembly.instances.is_flexible_assembly(key)  # Uses isRigid flag
]
```

### Parts as Single STL Files

Both regular parts and rigid subassemblies appear in `cad.parts` because both are downloaded as single STL files from Onshape. This unified treatment simplifies downstream mesh processing.

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
