# Migration Guide: Legacy to Type-Safe Assembly System

## Overview

This guide helps developers migrate from the legacy assembly system to the new type-safe KeyNamer and CAD assembly architecture. The new system provides better performance, type safety, and cleaner code organization.

## Key Changes Summary

### Before (Legacy System)

- Generic `assemblies` collection with runtime `isRigid` checks
- String-based name mapping with complex `sanitized_to_prefixed` conversions
- Boolean flags scattered throughout the codebase
- Runtime assembly classification

### After (New System)

- Type-safe `rigid_assemblies` and `flexible_assemblies` collections
- TypedKey system with compile-time type information
- Parse-time assembly classification based on `max_depth`
- Elimination of complex string manipulation

## API Migration

### Collection Access

#### Legacy Pattern

```python
# Old: Generic collection with runtime filtering
for key, assembly in cad.assemblies.items():
    if assembly.isRigid:
        process_rigid_assembly(assembly)
    else:
        process_flexible_assembly(assembly)

# Old: Manual filtering
rigid_assemblies = {k: v for k, v in cad.assemblies.items() if v.isRigid}
flexible_assemblies = {k: v for k, v in cad.assemblies.items() if not v.isRigid}
```

#### New Pattern

```python
# New: Direct type-safe access
for key, assembly in cad.rigid_assemblies.items():
    process_rigid_assembly(assembly)

for key, assembly in cad.flexible_assemblies.items():
    process_flexible_assembly(assembly)

# New: Collections are already separated
rigid_assemblies = cad.rigid_assemblies      # Direct access
flexible_assemblies = cad.flexible_assemblies # Direct access
```

### Lookup Methods

#### Legacy Pattern

```python
# Old: Generic lookup with type checking
assembly = cad.lookup_assembly(name)
if assembly and assembly.isRigid:
    process_rigid(assembly)
elif assembly:
    process_flexible(assembly)
```

#### New Pattern

```python
# New: Type-specific lookups
rigid_assembly = cad.lookup_rigid_assembly(name)
if rigid_assembly:
    process_rigid(rigid_assembly)

flexible_assembly = cad.lookup_flexible_assembly(name)
if flexible_assembly:
    process_flexible(flexible_assembly)

# Or use generic lookup (searches both)
assembly = cad.lookup_assembly(name)  # Returns first match from either type
```

### Key Creation

#### Legacy Pattern

```python
# Old: Generic assembly keys
key = TypedKey.for_assembly(path, assembly_id)
# Later: Check runtime flag
if assemblies[key].isRigid:
    # Handle as rigid
```

#### New Pattern

```python
# New: Type-specific key creation
if is_rigid:
    key = TypedKey.for_rigid_assembly(path, assembly_id)
    cad.rigid_assemblies[key] = assembly
else:
    key = TypedKey.for_flexible_assembly(path, assembly_id)
    cad.flexible_assemblies[key] = assembly
```

## Data Structure Migration

### CAD Class Updates

#### Legacy Structure

```python
@dataclass
class CAD:
    assemblies: dict[TypedKey, AssemblyInstance]  # Mixed rigid/flexible
    # ... other fields
```

#### New Structure

```python
@dataclass
class CAD:
    rigid_assemblies: dict[TypedKey, AssemblyInstance]     # Rigid only
    flexible_assemblies: dict[TypedKey, AssemblyInstance]  # Flexible only
    # ... other fields

    @property
    def assembly_keys(self) -> list[TypedKey]:
        """Combined keys for backward compatibility."""
        return list(self.rigid_assemblies.keys()) + list(self.flexible_assemblies.keys())
```

### Builder Updates

#### Legacy Builder Pattern

```python
# Old: Generic collection building
self.assemblies: dict[TypedKey, AssemblyInstance] = {}

# Later classification
for key, assembly in self.assemblies.items():
    assembly.isRigid = determine_rigidity(assembly)
```

#### New Builder Pattern

```python
# New: Type-aware collection building
self.rigid_assemblies: dict[TypedKey, AssemblyInstance] = {}
self.flexible_assemblies: dict[TypedKey, AssemblyInstance] = {}

# Classification at creation time
if is_rigid:
    key = TypedKey.for_rigid_assembly(path, assembly_id)
    self.rigid_assemblies[key] = assembly
else:
    key = TypedKey.for_flexible_assembly(path, assembly_id)
    self.flexible_assemblies[key] = assembly
```

## Performance Optimizations

### Elimination of Runtime Checks

#### Legacy Performance Issues

```python
# Old: Repeated runtime checks (expensive)
def process_assemblies(cad):
    for key, assembly in cad.assemblies.items():
        if assembly.isRigid:          # Runtime check for every assembly
            fetch_single_mass_property(assembly)
        else:
            fetch_detailed_mass_properties(assembly)
```

#### New Performance Benefits

```python
# New: Direct collection access (O(1))
def process_assemblies(cad):
    # Process rigid assemblies (single bodies)
    for key, assembly in cad.rigid_assemblies.items():
        fetch_single_mass_property(assembly)    # No runtime checks needed

    # Process flexible assemblies (articulated)
    for key, assembly in cad.flexible_assemblies.items():
        fetch_detailed_mass_properties(assembly)
```

### Cache Strategy Updates

#### Legacy Cache Management

```python
# Old: Mixed cache with type checking during lookup
cache = {}
for key, assembly in cad.assemblies.items():
    cache[assembly.name] = (key, assembly.isRigid)

# Lookup with type filtering
def lookup_rigid_assembly(name):
    if name in cache:
        key, is_rigid = cache[name]
        if is_rigid:
            return cad.assemblies[key]
    return None
```

#### New Cache Management

```python
# New: Type-specific caches
cad.key_namer.build_reverse_cache(cad.rigid_assembly_keys + cad.flexible_assembly_keys)

# Direct type-safe lookup
def lookup_rigid_assembly(name):
    key = cad.key_namer.lookup_key(name, EntityType.RIGID_ASSEMBLY)
    return cad.rigid_assemblies.get(key) if key else None
```

## String Mapping Elimination

### Legacy Complex String Mapping

The old system used complex string manipulation:

```python
# Old: Complex string-based mapping system
rigid_subassembly_sanitized_to_prefixed = {}
for prefixed_key in rigid_subassemblies:
    sanitized_name = prefixed_key.split(SUBASSEMBLY_JOINER)[-1]
    rigid_subassembly_sanitized_to_prefixed[sanitized_name] = prefixed_key

# Later usage with complex lookups
if occurrence_name in rigid_subassembly_sanitized_to_prefixed:
    prefixed_name = rigid_subassembly_sanitized_to_prefixed[occurrence_name]
    assembly = rigid_subassemblies[prefixed_name]
```

### New Type-Safe Approach

The new system eliminates string manipulation:

```python
# New: Direct type-safe lookup
rigid_assembly = cad.lookup_rigid_assembly(occurrence_name)
if rigid_assembly:
    # Process directly - no string manipulation needed
    process_rigid_assembly(rigid_assembly)
```

## Code Examples: Step-by-Step Migration

### Example 1: Assembly Processing Function

#### Before Migration

```python
def process_all_assemblies(cad: CAD):
    """Legacy assembly processing with runtime checks."""
    rigid_count = 0
    flexible_count = 0

    for key, assembly in cad.assemblies.items():
        if assembly.isRigid:
            rigid_count += 1
            # Process as single body
            mass_props = fetch_combined_mass_properties(assembly)
            create_single_urdf_link(assembly, mass_props)
        else:
            flexible_count += 1
            # Process with internal structure
            parts = get_internal_parts(assembly)
            mates = get_internal_mates(assembly)
            create_articulated_urdf_structure(parts, mates)

    LOGGER.info(f"Processed {rigid_count} rigid and {flexible_count} flexible assemblies")
```

#### After Migration

```python
def process_all_assemblies(cad: CAD):
    """Type-safe assembly processing with direct access."""
    # Process rigid assemblies
    for key, assembly in cad.rigid_assemblies.items():
        # Process as single body
        mass_props = fetch_combined_mass_properties(assembly)
        create_single_urdf_link(assembly, mass_props)

    # Process flexible assemblies
    for key, assembly in cad.flexible_assemblies.items():
        # Process with internal structure
        parts = get_internal_parts(assembly)
        mates = get_internal_mates(assembly)
        create_articulated_urdf_structure(parts, mates)

    LOGGER.info(f"Processed {len(cad.rigid_assemblies)} rigid and "
                f"{len(cad.flexible_assemblies)} flexible assemblies")
```

### Example 2: Graph Construction

#### Before Migration

```python
def build_assembly_graph(cad: CAD) -> nx.DiGraph:
    """Legacy graph construction with runtime classification."""
    graph = nx.DiGraph()

    for key, assembly in cad.assemblies.items():
        if assembly.isRigid:
            # Add as single node
            graph.add_node(key, type="rigid_body", assembly=assembly)
        else:
            # Add internal structure
            internal_graph = build_internal_graph(assembly)
            graph = nx.compose(graph, internal_graph)

    return graph
```

#### After Migration

```python
def build_assembly_graph(cad: CAD) -> nx.DiGraph:
    """Type-safe graph construction with direct classification."""
    graph = nx.DiGraph()

    # Add rigid assemblies as single nodes
    for key, assembly in cad.rigid_assemblies.items():
        graph.add_node(key, type="rigid_body", assembly=assembly)

    # Add flexible assemblies with internal structure
    for key, assembly in cad.flexible_assemblies.items():
        internal_graph = build_internal_graph(assembly)
        graph = nx.compose(graph, internal_graph)

    return graph
```

### Example 3: Mass Property Fetching

#### Before Migration

```python
async def fetch_mass_properties(cad: CAD, client):
    """Legacy mass property fetching with filtering."""
    rigid_assemblies = []
    flexible_parts = []

    # Filter assemblies by type
    for key, assembly in cad.assemblies.items():
        if assembly.isRigid:
            rigid_assemblies.append(assembly)

    # Get parts from flexible assemblies only
    for key, part in cad.parts.items():
        parent_assembly = find_parent_assembly(key, cad.assemblies)
        if parent_assembly and not parent_assembly.isRigid:
            flexible_parts.append(part)

    # Fetch mass properties
    await fetch_rigid_mass_properties(rigid_assemblies, client)
    await fetch_part_mass_properties(flexible_parts, client)
```

#### After Migration

```python
async def fetch_mass_properties(cad: CAD, client):
    """Type-safe mass property fetching with direct access."""
    # Fetch for rigid assemblies
    rigid_assemblies = list(cad.rigid_assemblies.values())
    await fetch_rigid_mass_properties(rigid_assemblies, client)

    # Get parts from flexible assemblies only
    flexible_parts = []
    for key, part in cad.parts.items():
        if is_part_in_flexible_assembly(key, cad):
            flexible_parts.append(part)

    await fetch_part_mass_properties(flexible_parts, client)

def is_part_in_flexible_assembly(part_key: TypedKey, cad: CAD) -> bool:
    """Helper to check if part is in a flexible assembly."""
    # Check if any parent assembly is flexible
    parent_path = part_key.parent_path
    while parent_path:
        # Check if this path corresponds to a flexible assembly
        for assembly_key in cad.flexible_assembly_keys:
            if assembly_key.path == parent_path:
                return True
        parent_path = parent_path[:-1]
    return False
```

## Testing Migration

### Validation Strategy

1. **Parallel Execution**: Run both old and new systems side-by-side
2. **Data Validation**: Compare outputs for consistency
3. **Performance Measurement**: Verify performance improvements

```python
def validate_migration(assembly: Assembly, client, max_depth: int):
    """Validate migration by comparing old and new systems."""

    # Legacy system (if still available)
    legacy_cad = build_legacy_cad(assembly, client, max_depth)

    # New system
    new_cad = CAD.from_assembly(assembly, client, max_depth)

    # Validate assembly counts
    legacy_rigid = sum(1 for a in legacy_cad.assemblies.values() if a.isRigid)
    legacy_flexible = sum(1 for a in legacy_cad.assemblies.values() if not a.isRigid)

    assert len(new_cad.rigid_assemblies) == legacy_rigid
    assert len(new_cad.flexible_assemblies) == legacy_flexible

    # Validate total counts
    assert len(new_cad.parts) == len(legacy_cad.parts)
    assert len(new_cad.assembly_keys) == len(legacy_cad.assemblies)

    LOGGER.info("Migration validation passed!")
```

### Performance Benchmarking

```python
import time
from typing import Callable

def benchmark_migration(assembly: Assembly, client, max_depth: int):
    """Benchmark performance improvements."""

    def time_function(func: Callable, *args) -> tuple[float, any]:
        start = time.time()
        result = func(*args)
        end = time.time()
        return end - start, result

    # Benchmark assembly processing
    cad = CAD.from_assembly(assembly, client, max_depth)

    # Old approach simulation
    old_time, _ = time_function(process_assemblies_legacy_style, cad)

    # New approach
    new_time, _ = time_function(process_assemblies_new_style, cad)

    improvement = ((old_time - new_time) / old_time) * 100
    LOGGER.info(f"Performance improvement: {improvement:.1f}% faster")

def process_assemblies_legacy_style(cad: CAD):
    """Simulate legacy processing with runtime checks."""
    for key in cad.assembly_keys:
        # Simulate lookup overhead
        if key.entity_type == EntityType.RIGID_ASSEMBLY:
            assembly = cad.rigid_assemblies[key]
            if hasattr(assembly, 'isRigid') and assembly.isRigid:  # Simulated check
                process_rigid(assembly)
        else:
            assembly = cad.flexible_assemblies[key]
            process_flexible(assembly)

def process_assemblies_new_style(cad: CAD):
    """New type-safe processing."""
    for key, assembly in cad.rigid_assemblies.items():
        process_rigid(assembly)

    for key, assembly in cad.flexible_assemblies.items():
        process_flexible(assembly)
```

## Common Migration Pitfalls

### 1. Assuming Backward Compatibility

**Pitfall**: Expecting `cad.assemblies` to still exist

```python
# This will fail after migration
for key, assembly in cad.assemblies.items():  # AttributeError
    process_assembly(assembly)
```

**Solution**: Use type-specific collections

```python
# Correct approach
for key, assembly in cad.rigid_assemblies.items():
    process_assembly(assembly)
for key, assembly in cad.flexible_assemblies.items():
    process_assembly(assembly)
```

### 2. Mixing Entity Types in Collections

**Pitfall**: Trying to store mixed types in one collection

```python
# Don't do this
all_assemblies = {}
all_assemblies.update(cad.rigid_assemblies)    # Wrong: mixed TypedKey types
all_assemblies.update(cad.flexible_assemblies)
```

**Solution**: Use the provided combined property

```python
# Correct approach
all_assembly_keys = cad.assembly_keys  # Returns combined list of keys
```

### 3. Ignoring Type Information

**Pitfall**: Not leveraging type information for optimization

```python
# Inefficient: Still using runtime checks
for key in cad.assembly_keys:
    if key.entity_type == EntityType.RIGID_ASSEMBLY:
        assembly = cad.rigid_assemblies[key]
        # Process...
```

**Solution**: Use direct iteration

```python
# Efficient: Direct type-safe iteration
for key, assembly in cad.rigid_assemblies.items():
    # Process rigid assemblies
```

## Migration Checklist

- [ ] Update collection access patterns (`assemblies` → `rigid_assemblies` + `flexible_assemblies`)
- [ ] Replace runtime `isRigid` checks with type-based logic
- [ ] Update TypedKey creation to use type-specific factory methods
- [ ] Migrate lookup methods to type-specific versions
- [ ] Remove complex string mapping logic where applicable
- [ ] Update cache building to use type-separated collections
- [ ] Test performance improvements
- [ ] Validate output consistency
- [ ] Update documentation and examples

## Next Steps

After completing the migration:

1. **Remove Legacy Code**: Clean up old boolean flags and generic collections
2. **Optimize Further**: Look for additional opportunities to leverage type information
3. **Extend Type System**: Consider adding more entity types for future features
4. **Update Documentation**: Ensure all examples use the new patterns

The new type-safe system provides a solid foundation for building more robust and performant CAD processing pipelines.
