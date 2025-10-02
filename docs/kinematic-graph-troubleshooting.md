# KinematicGraph Troubleshooting Guide

## Common Issues and Solutions

### Issue 1: PathKey Comparison Error

**Error:**

```
TypeError: '<' not supported between instances of 'PathKey' and 'PathKey'
```

**Cause:** PathKey didn't implement comparison operators needed for sorting in graph visualization.

**Solution:** Added comparison methods to PathKey class ([parse.py:124-158](../onshape_robotics_toolkit/parse.py#L124)):

- `__lt__`: Less-than comparison
- `__le__`: Less-than-or-equal
- `__gt__`: Greater-than
- `__ge__`: Greater-than-or-equal

PathKeys are sorted by depth first (shallower parts first), then lexicographically by path elements.

### Issue 2: Invalid PathKeys in Mates

**Warning:**

```
WARNING: Part PathKey(...) involved in mate but not found in parts registry
```

**Cause:** `process_mates_and_relations()` creates mate entries with PathKeys that don't correspond to actual instances in the registries. This can happen when:

- Mates reference parts that weren't populated (missing from assembly JSON)
- Mate processing creates incorrect PathKey mappings
- Parts are in flexible subassemblies that weren't fetched

**Solution:** Added mate validation in KinematicGraph ([graph.py:136-210](../onshape_robotics_toolkit/graph.py#L136)):

- `_validate_mates()`: Filters out mates with invalid PathKeys
- `_is_valid_mate_target()`: Checks if PathKey exists in registries
- Validates against: parts registry, part instances, rigid assembly instances

### Issue 3: User-Defined Root Not in Graph

**Error:**

```
KeyError: PathKey(('M0cLO6yVimMv6KhRM',))
```

**Cause:** User-defined root part gets filtered out when removing disconnected subgraphs or invalid mates.

**Solution:** Added root validation ([graph.py:125-131](../onshape_robotics_toolkit/graph.py#L125)):

```python
if user_defined_root and user_defined_root not in main_graph.nodes:
    LOGGER.warning(
        f"User-defined root {user_defined_root} not in main graph after filtering. "
        f"Will use centrality-based root detection."
    )
    user_defined_root = None
```

### Issue 4: Flexible Assemblies in Mates

**Warning:**

```
WARNING: Part PathKey(...) involved in mate but not found in parts registry
```

**Cause:** Mates may reference flexible assembly instances, which shouldn't be nodes (only parts within them should be nodes).

**Solution:** Enhanced node addition logic ([graph.py:181-232](../onshape_robotics_toolkit/graph.py#L181)):

- Checks if PathKey is a flexible assembly
- Skips flexible assemblies (logs debug message)
- Only adds parts and rigid assemblies as nodes

## Debugging Workflow

When KinematicGraph has issues, check in this order:

### 1. Check Assembly Structure

```python
cad = CAD.from_assembly(assembly, max_depth=1)
print(f"Parts: {len(cad.parts)}")
print(f"Part instances: {len(cad.root_assembly.instances.parts)}")
print(f"Assembly instances: {len(cad.root_assembly.instances.assemblies)}")
```

### 2. Check Mate Processing

```python
cad.process_mates_and_relations()
print(f"Root mates: {len(cad.root_assembly.mates.mates)}")
for key, data in cad.sub_assemblies.items():
    print(f"Subassembly {key} mates: {len(data.mates.mates)}")
```

### 3. Check Mate Validity

```python
tree = KinematicGraph(cad)  # Will log warnings about invalid mates
```

### 4. Examine Graph Structure

```python
print(f"Nodes: {len(tree.graph.nodes)}")
print(f"Edges: {len(tree.graph.edges)}")
print(f"Root: {tree.root_node}")

# Check for disconnected components
undirected = tree.graph.to_undirected()
components = list(nx.connected_components(undirected))
print(f"Connected components: {len(components)}")
```

## Understanding Warnings

### "Filtered out N mates with invalid PathKeys"

**What it means:** Some mates reference PathKeys that don't exist in any registry.

**Why it happens:**

- Parts not populated from assembly JSON
- Mate processing created incorrect PathKey mappings
- References to parts in unfetched flexible subassemblies

**What to do:**

1. Check if parts are missing from assembly JSON
2. Verify `max_depth` setting allows proper subassembly expansion
3. Ensure `process_mates_and_relations()` is called before creating tree
4. If many mates are filtered, assembly data may be incomplete

### "Graph has one or more unconnected subgraphs"

**What it means:** The kinematic graph has multiple disconnected components.

**Why it happens:**

- Multiple independent assemblies in the document
- Invalid mates filtered out, breaking connections
- Parts involved in mates but not connected to main structure

**What to do:**

1. Review "Original graph structure" log to see components
2. Check if filtered mates are breaking connections
3. Verify assembly has proper mate relationships
4. Consider if multiple assemblies are intentional

### "User-defined root not in main graph after filtering"

**What it means:** The part marked as fixed in Onshape was removed from the graph.

**Why it happens:**

- Fixed part has no valid mate relationships
- Fixed part's mates were all filtered as invalid
- Fixed part in disconnected subgraph that was removed

**What to do:**

1. Verify fixed part has valid mates in assembly
2. Check if fixed part's mates reference valid parts
3. Tree will use centrality-based root detection as fallback
4. Consider manually specifying a different root

## Best Practices

### 1. Always Call process_mates_and_relations()

```python
cad = CAD.from_assembly(assembly, max_depth=1)
cad.process_mates_and_relations()  # Required!
tree = KinematicGraph(cad)
```

### 2. Handle Empty Trees

```python
tree = KinematicGraph(cad)
if tree.root_node is None:
    print("Warning: KinematicGraph is empty or invalid")
    return
```

### 3. Check for Disconnected Subgraphs

```python
if tree.topological_order is None:
    print("Warning: Graph has cycles, no topological order")
```

### 4. Use Appropriate max_depth

- `max_depth=0`: All assemblies rigid (fastest, minimal detail)
- `max_depth=1`: Depth-1 assemblies flexible (balanced)
- `max_depth=∞`: All assemblies flexible (slowest, maximum detail)

## Performance Tips

### Large Assemblies

- Use lower `max_depth` to reduce API calls
- Rigid assemblies are treated as single parts (better performance)
- Consider filtering out hidden parts before tree creation

### Many Mates

- Invalid mates are filtered efficiently (no node creation)
- Validation happens once during tree construction
- Graph operations are O(N+E) where N=nodes, E=edges

## Related Documentation

- [KinematicGraph Migration Guide](kinematic-tree-migration.md)
- [Developing Documentation](developing.md)
- [CAD Assembly System](cad-assembly-system.md)
