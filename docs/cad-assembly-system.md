# CAD Assembly System Architecture

## Overview

The CAD assembly system provides a comprehensive framework for parsing, classifying, and managing Onshape assembly data. It features type-safe assembly classification, efficient single-traversal parsing, and clean separation between rigid and flexible assemblies for optimal robot pipeline integration.

## Core Architecture

### CAD Class

The `CAD` class serves as the central container for all parsed assembly data, organized by entity type using TypedKey identification.

```python
@dataclass
class CAD:
    assembly: Assembly                                      # Original Onshape assembly
    instances: dict[TypedKey, Union[PartInstance, AssemblyInstance]]  # All instances
    parts: dict[TypedKey, PartInstance]                    # Part instances only
    rigid_assemblies: dict[TypedKey, AssemblyInstance]     # Rigid assembly instances
    flexible_assemblies: dict[TypedKey, AssemblyInstance]  # Flexible assembly instances
    occurrences: dict[TypedKey, Occurrence]               # All occurrences
    features: dict[TypedKey, AssemblyFeature]             # Assembly features
    mates: dict[TypedKey, MateFeatureData]                # Mate features
    relations: dict[TypedKey, MateRelationFeatureData]    # Mate relations
    id_resolver: IdToNameResolver                          # ID resolution system
    key_namer: KeyNamer                                   # Type-aware naming system
    subassemblies: dict[TypedKey, SubAssembly]            # Flexible subassemblies
    rigid_subassemblies: dict[TypedKey, RootAssembly]     # Rigid subassemblies
    max_depth: int                                        # Classification depth parameter
```

### Assembly Classification Strategy

The system automatically classifies assemblies as rigid or flexible based on the `max_depth` parameter during parse-time:

#### Classification Rules

```python
if max_depth == 0:
    # All assemblies are rigid (maximum performance)
    is_rigid = True
else:
    # Assemblies beyond max_depth are rigid
    is_rigid = instance_depth > max_depth
```

#### Practical Examples

- `max_depth=0`: All subassemblies → rigid (fastest, single bodies)
- `max_depth=1`: Root subassemblies → flexible, nested → rigid
- `max_depth=∞`: All subassemblies → flexible (full articulation)

## AssemblyStructureBuilder

Single-traversal builder that efficiently constructs all assembly mappings.

### Key Features

- **Single-pass construction**: Builds all mappings in one efficient traversal
- **Type-based classification**: Determines rigidity at parse-time
- **Hierarchical processing**: Handles nested assembly structures
- **Memory efficient**: Avoids redundant data structures

### Construction Process

```python
class AssemblyStructureBuilder:
    def build(self) -> CAD:
        self._traverse_and_build()    # Single traversal for all entities
        self._build_occurrences()     # Build occurrence mappings
        return CAD(...)               # Return populated CAD container
```

#### Phase 1: Instance and Feature Processing

```python
def _process_assembly_level(self, assembly_level, path, is_rigid=False):
    # Process instances
    for instance in assembly_level.instances:
        if instance.type == InstanceType.PART:
            key = TypedKey.for_part(instance_path, instance.id)
            self.parts[key] = instance

        elif instance.type == InstanceType.ASSEMBLY:
            # Determine rigidity based on depth
            if is_rigid or (self.max_depth > 0 and len(instance_path) > self.max_depth):
                key = TypedKey.for_rigid_assembly(instance_path, instance.id)
                self.rigid_assemblies[key] = instance
            else:
                key = TypedKey.for_flexible_assembly(instance_path, instance.id)
                self.flexible_assemblies[key] = instance

    # Process features (mates, relations)
    for feature in assembly_level.features:
        # Classify and store features by type
```

#### Phase 2: Hierarchical Traversal

```python
def _traverse_and_build(self):
    # Process root assembly
    self._process_assembly_level(self.assembly.rootAssembly, path=())

    # Process subassemblies breadth-first
    subassembly_queue = deque()
    # ... populate queue with root-level assemblies

    while subassembly_queue:
        path, assembly_instance, subassembly = subassembly_queue.popleft()

        # Determine rigidity for this level
        is_rigid = self.max_depth == 0 or len(path) > self.max_depth

        # Process this subassembly level
        self._process_assembly_level(subassembly, path, is_rigid, subassembly)

        # Add nested assemblies to queue for further processing
```

## CAD Class API

### Type-Safe Assembly Access

```python
# Access assemblies by classification
cad.rigid_assemblies: dict[TypedKey, AssemblyInstance]     # Single bodies
cad.flexible_assemblies: dict[TypedKey, AssemblyInstance]  # Articulated assemblies

# Get keys by type
cad.rigid_assembly_keys: list[TypedKey]
cad.flexible_assembly_keys: list[TypedKey]
cad.assembly_keys: list[TypedKey]  # Combined rigid + flexible keys
```

### Lookup Methods

```python
# Type-specific lookups
rigid_assembly = cad.lookup_rigid_assembly(name)
flexible_assembly = cad.lookup_flexible_assembly(name)

# Generic lookup (searches both types)
assembly = cad.lookup_assembly(name)

# Other entity lookups
part = cad.lookup_part(name)
mate = cad.lookup_mate(name)
subassembly = cad.lookup_subassembly(name)
```

### Cache Management

```python
# Build reverse lookup cache
cad.build_reverse_cache()

# Prefixed cache for subassembly contexts
cad.build_reverse_cache(prefix="subassembly_prefix")
```

### Tree Visualization

```python
# Display hierarchical structure with type information
cad.show_tree()
```

Example output:

```
Assembly Root
|-- Part 1 <1> (PART)
|-- Assembly 2 <1> (FLEXIBLE_ASSEMBLY)
    |-- Part 12 <1> (PART)
    |-- Assembly 1 <1> (RIGID_ASSEMBLY) [RIGID]
        |-- Part 10 <1> (PART)
```

## Factory Method

### CAD.from_assembly()

Primary entry point for creating CAD instances from Onshape assemblies.

```python
@classmethod
def from_assembly(cls, assembly: Assembly, client, max_depth: int = 0) -> CAD:
    """
    Create CAD from Onshape assembly with full parsing.

    Args:
        assembly: Onshape assembly to parse
        client: Onshape client for fetching subassemblies
        max_depth: Maximum depth for flexible classification

    Returns:
        Fully parsed CAD with type-classified assemblies
    """
```

#### Usage Examples

```python
# Maximum performance: All assemblies as rigid bodies
cad = CAD.from_assembly(assembly, client, max_depth=0)

# Balanced: Root assemblies flexible, deeper ones rigid
cad = CAD.from_assembly(assembly, client, max_depth=1)

# Full articulation: All assemblies flexible
cad = CAD.from_assembly(assembly, client, max_depth=float('inf'))
```

## Integration with Robot Pipeline

### Rigid Assembly Processing

Rigid assemblies are treated as single bodies for performance optimization:

```python
def process_rigid_assemblies(cad: CAD):
    for key, rigid_assembly in cad.rigid_assemblies.items():
        # Fetch as single body with combined mass properties
        combined_mass = fetch_combined_mass_properties(rigid_assembly)

        # Generate single URDF link
        link = create_urdf_link(rigid_assembly, combined_mass)

        # No internal joints - treated as monolithic body
```

### Flexible Assembly Processing

Flexible assemblies maintain internal joint structure:

```python
def process_flexible_assemblies(cad: CAD):
    for key, flexible_assembly in cad.flexible_assemblies.items():
        # Process internal structure
        internal_parts = get_internal_parts(flexible_assembly)
        internal_mates = get_internal_mates(flexible_assembly)

        # Generate URDF links and joints for internal structure
        for part in internal_parts:
            link = create_urdf_link(part)

        for mate in internal_mates:
            joint = create_urdf_joint(mate)
```

### Graph Construction

The type-safe system enables optimized graph construction:

```python
def build_kinematic_graph(cad: CAD) -> nx.DiGraph:
    graph = nx.DiGraph()

    # Add rigid assemblies as single nodes
    for key, rigid_assembly in cad.rigid_assemblies.items():
        graph.add_node(key, entity_type="rigid_body", instance=rigid_assembly)

    # Add flexible assemblies with internal structure
    for key, flexible_assembly in cad.flexible_assemblies.items():
        subgraph = build_internal_graph(flexible_assembly)
        graph = nx.compose(graph, subgraph)

    # Connect assemblies through mates
    for key, mate in cad.mates.items():
        add_mate_edge(graph, mate)

    return graph
```

## Performance Optimizations

### Single-Traversal Construction

The builder constructs all data structures in one pass:

```python
# Traditional approach: Multiple passes
parts = extract_parts(assembly)           # Pass 1
assemblies = extract_assemblies(assembly) # Pass 2
features = extract_features(assembly)     # Pass 3
# ... multiple API calls and traversals

# Optimized approach: Single pass
builder = AssemblyStructureBuilder(assembly, max_depth)
cad = builder.build()  # Everything in one traversal
```

### Type-Specific Collections

Direct access to typed collections eliminates filtering overhead:

```python
# Old approach: Runtime filtering
rigid_assemblies = [a for a in assemblies if a.isRigid]

# New approach: Direct access
rigid_assemblies = cad.rigid_assemblies  # O(1) access
```

### Efficient Mass Property Fetching

Only fetch mass properties for kinematically relevant parts:

```python
async def fetch_mass_properties(cad: CAD, client):
    # Skip mass properties for parts in rigid assemblies (performance)
    kinematic_parts = []

    for key, part in cad.parts.items():
        # Only fetch for parts in flexible assemblies
        if is_part_in_flexible_assembly(key, cad):
            kinematic_parts.append(part)

    # Fetch mass properties only for kinematic parts
    await fetch_mass_properties_batch(kinematic_parts, client)
```

## Migration Patterns

### From Legacy Boolean Checks

```python
# Legacy pattern
for assembly in assemblies:
    if assembly.isRigid:
        process_as_rigid(assembly)
    else:
        process_as_flexible(assembly)

# New pattern
for key, assembly in cad.rigid_assemblies.items():
    process_as_rigid(assembly)

for key, assembly in cad.flexible_assemblies.items():
    process_as_flexible(assembly)
```

### From Generic Collections

```python
# Legacy pattern
assemblies = cad.get_all_assemblies()
filtered_rigid = filter_by_rigidity(assemblies, rigid=True)

# New pattern
rigid_assemblies = cad.rigid_assemblies  # Direct access
```

### From String-Based Lookups

```python
# Legacy pattern
assembly_name = parse_occurrence_string(occurrence)
if assembly_name in rigid_assembly_mapping:
    prefixed_name = rigid_assembly_mapping[assembly_name]
    assembly = assemblies[prefixed_name]

# New pattern
assembly = cad.lookup_rigid_assembly(assembly_name)  # Type-safe
```

## Best Practices

### 1. Choose Appropriate max_depth

```python
# For performance-critical applications
cad = CAD.from_assembly(assembly, client, max_depth=0)

# For balanced performance/fidelity
cad = CAD.from_assembly(assembly, client, max_depth=2)

# For maximum fidelity
cad = CAD.from_assembly(assembly, client, max_depth=float('inf'))
```

### 2. Use Type-Specific Access

```python
# Good: Type-specific access
for key, rigid_assembly in cad.rigid_assemblies.items():
    # Process as single body

# Avoid: Generic access with runtime checks
for key, assembly in cad.instances.items():
    if isinstance(assembly, AssemblyInstance) and assembly.isRigid:
        # Runtime overhead
```

### 3. Leverage Hierarchical Information

```python
# Use TypedKey path information
def process_by_depth(cad: CAD):
    for key, part in cad.parts.items():
        if key.depth > 3:
            # Handle deeply nested parts differently
            optimize_deep_part(part)
```

### 4. Build Caches Strategically

```python
# Build reverse cache once after construction
cad = CAD.from_assembly(assembly, client, max_depth=1)
cad.build_reverse_cache()

# Then use efficient lookups throughout pipeline
```

## Future Enhancements

### Advanced Classification

```python
# Potential future classification strategies
class AssemblyClassifier:
    def classify_by_feature_complexity(self, assembly) -> EntityType:
        """Classify based on internal feature complexity."""

    def classify_by_mass_distribution(self, assembly) -> EntityType:
        """Classify based on mass property analysis."""

    def classify_by_joint_count(self, assembly) -> EntityType:
        """Classify based on internal joint count."""
```

### Dynamic Reclassification

```python
# Potential future capability
def reclassify_assembly(cad: CAD, key: TypedKey, new_type: EntityType):
    """Dynamically reclassify assembly based on analysis results."""
    # Move between rigid_assemblies and flexible_assemblies
    # Update dependent structures
    # Regenerate affected caches
```

## Error Handling

### Common Issues and Solutions

```python
# Handle missing subassemblies gracefully
try:
    cad = CAD.from_assembly(assembly, client, max_depth=1)
except SubassemblyFetchError as e:
    LOGGER.warning(f"Failed to fetch subassembly: {e}")
    # Fallback to rigid classification
    cad = CAD.from_assembly(assembly, client, max_depth=0)

# Handle duplicate names in cache
cad.key_namer.build_reverse_cache(all_keys)  # Warnings logged automatically

# Validate classification results
assert len(cad.rigid_assemblies) + len(cad.flexible_assemblies) == expected_count
```

The CAD assembly system provides a robust, type-safe foundation for processing complex Onshape assemblies while maintaining optimal performance through intelligent classification and efficient data structures.
