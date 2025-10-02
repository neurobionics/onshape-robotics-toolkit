# KinematicTree Migration Guide

This guide explains the new `KinematicTree` class and how it replaces the old `create_graph()` function.

## Overview

The new `KinematicTree` class uses the PathKey-based CAD system for improved type safety, performance, and maintainability.

### Old System (String-Based)

```python
# Old approach with string keys
occurrences = get_occurrences(assembly)
instances = get_instances(assembly)
parts = get_parts(assembly, client)
mates = get_mates(assembly)

# Create graph with string-based keys joined by MATE_JOINER
graph, root_node = create_graph(
    occurrences,
    instances,
    parts,
    mates,
    use_user_defined_root=True
)

# Node is a string like "part1_ABC123_part2"
```

### New System (PathKey-Based)

```python
# New approach with PathKey system
cad = CAD.from_assembly(assembly, max_depth=1)

# Create kinematic tree with PathKey nodes
tree = KinematicTree(cad, use_user_defined_root=True)

# Node is a PathKey: PathKey(("M0cLO6yVimMv6KhRM", "MZHBlAU4IxmX6u6A0"))
```

## Key Improvements

### 1. Type Safety

**Old:** String-based keys with runtime joining

```python
mate_key = f"{parent}{MATE_JOINER}{child}"  # String concatenation
```

**New:** Immutable PathKey with compile-time safety

```python
mate_key = (parent_key, child_key)  # Tuple of PathKeys
```

### 2. Simplified Data Flow

**Old:** Multiple separate dictionaries to manage

```python
occurrences = get_occurrences(assembly)
instances = get_instances(assembly)
parts = get_parts(assembly, client)
mates = get_mates(assembly)
graph, root_node = create_graph(occurrences, instances, parts, mates)
```

**New:** Single CAD object with registries

```python
cad = CAD.from_assembly(assembly, max_depth=1)
tree = KinematicTree(cad, use_user_defined_root=True)
```

### 3. Automatic Mate Collection

**Old:** Manual mate collection from multiple sources

```python
mates = get_mates(assembly)  # Only root mates?
# Subassembly mates handled separately
```

**New:** Automatic aggregation from root and subassemblies

```python
# KinematicTree automatically collects mates from:
# - cad.root_assembly.mates
# - All cad.sub_assemblies[key].mates
all_mates = tree._collect_all_mates()
```

### 4. Hierarchical Path Information

**Old:** Flattened string keys lose hierarchy

```python
node = "rigid_sub_ABC123_level1_DEF456_part"  # Hard to parse depth
```

**New:** PathKey preserves hierarchy

```python
node = PathKey(("rigid_sub", "level1", "part"))
depth = node.depth  # 3
parent = node.parent_key  # PathKey(("rigid_sub", "level1"))
```

## API Comparison

### Creating a Graph/Tree

**Old:**

```python
graph, root_node = create_graph(
    occurrences=occurrences,
    instances=instances,
    parts=parts,
    mates=mates,
    use_user_defined_root=True
)
```

**New:**

```python
tree = KinematicTree(
    cad=cad,
    use_user_defined_root=True
)
# Access graph: tree.graph
# Access root: tree.root_node
```

### Accessing Nodes

**Old:**

```python
# Node is a string
for node in graph.nodes:
    part_data = parts[node]  # May fail if key format differs
```

**New:**

```python
# Node is a PathKey
for node in tree.graph.nodes:
    part_data = cad.parts[node]  # Type-safe lookup
    metadata = tree.get_node_metadata(node)  # Helper method
```

### Navigating the Tree

**Old:**

```python
children = list(graph.successors(node))
parents = list(graph.predecessors(node))
```

**New:**

```python
children = tree.get_children(node)  # Convenience method
parent = tree.get_parent(node)  # Returns single parent or None
```

### Getting Mate Data

**Old:**

```python
# Mate data stored in edge attributes
if graph.has_edge(parent, child):
    mate_data = mates[f"{parent}{MATE_JOINER}{child}"]  # Manual lookup
```

**New:**

```python
# Mate data stored in edge and accessible via helper
mate_data = tree.get_mate_data(parent_key, child_key)
if mate_data:
    print(mate_data.featureType)
```

### Topological Ordering

**Old:**

```python
order = get_topological_order(graph)
if order:
    for node in order:
        # Process in order
```

**New:**

```python
# Automatically computed during tree construction
if tree.topological_order:
    for node in tree.topological_order:
        # Process in order
```

## Migration Checklist

When migrating from `create_graph()` to `KinematicTree`:

- [ ] Replace separate data fetching with `CAD.from_assembly()`
- [ ] Change `create_graph()` call to `KinematicTree()` constructor
- [ ] Update node handling from strings to PathKeys
- [ ] Use `tree.graph` instead of `graph` variable
- [ ] Use `tree.root_node` instead of `root_node` variable
- [ ] Replace `graph.successors()` with `tree.get_children()`
- [ ] Replace `graph.predecessors()` with `tree.get_parent()`
- [ ] Update mate lookups to use `tree.get_mate_data()`
- [ ] Use `tree.topological_order` instead of `get_topological_order(graph)`

## Example Migration

### Before (Old System)

```python
from onshape_robotics_toolkit.parse import (
    get_occurrences,
    get_instances,
    get_parts,
    get_mates,
)
from onshape_robotics_toolkit.graph import create_graph

# Fetch assembly
assembly = client.get_assembly(doc_id, wvm_type, wvm_id, element_id)

# Create separate data structures
occurrences = get_occurrences(assembly)
instances = get_instances(assembly)
parts = get_parts(assembly, client)
mates = get_mates(assembly)

# Create graph
graph, root_node = create_graph(
    occurrences,
    instances,
    parts,
    mates,
    use_user_defined_root=True
)

# Navigate graph
print(f"Root: {root_node}")
for child in graph.successors(root_node):
    print(f"Child: {child}")
```

### After (New System)

```python
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.graph import KinematicTree

# Fetch assembly
assembly = client.get_assembly(doc_id, wvm_type, wvm_id, element_id)

# Create CAD document
cad = CAD.from_assembly(assembly, max_depth=1)

# Create kinematic tree
tree = KinematicTree(cad, use_user_defined_root=True)

# Navigate tree
print(f"Root: {tree.root_node}")
for child in tree.get_children(tree.root_node):
    print(f"Child: {child}")

# Access metadata
metadata = tree.get_node_metadata(tree.root_node)
print(f"Root part name: {metadata.get('name')}")
```

## Benefits Summary

1. **Type Safety**: PathKey is strongly typed vs. strings
2. **Cleaner API**: Single CAD object instead of 4 separate dicts
3. **Better Organization**: Tree class encapsulates graph operations
4. **Automatic Aggregation**: Mates collected from all subassemblies
5. **Rich Metadata**: Easy access to node and edge data
6. **Convenience Methods**: `get_children()`, `get_parent()`, `get_mate_data()`
7. **Hierarchy Preservation**: PathKey maintains depth and parent relationships

## Backward Compatibility

The old `create_graph()` function remains available for backward compatibility but is deprecated. New code should use `KinematicTree`.

```python
# Old function still works but deprecated
from onshape_robotics_toolkit.graph import create_graph

# Use KinematicTree for new code
from onshape_robotics_toolkit.graph import KinematicTree
```
