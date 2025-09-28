# KeyNamer and ID Resolution System

## Overview

The KeyNamer system provides type-safe, hierarchical naming and lookup for CAD assembly entities. It consists of three main components that work together to create a robust entity identification and resolution system:

1. **TypedKey**: Type-safe hierarchical keys with entity context
2. **IdToNameResolver**: Centralized ID-to-name mapping for efficient lookups
3. **KeyNamer**: Type-aware naming with reverse lookup capabilities

## Core Components

### EntityType Enum

```python
class EntityType(Enum):
    PART = "PART"
    RIGID_ASSEMBLY = "RIGID_ASSEMBLY"
    FLEXIBLE_ASSEMBLY = "FLEXIBLE_ASSEMBLY"
    FEATURE = "FEATURE"
    OCCURRENCE = "OCCURRENCE"
```

**Key Design Decision**: Separated assemblies into `RIGID_ASSEMBLY` and `FLEXIBLE_ASSEMBLY` types to enable type-safe classification at parse-time rather than runtime boolean checks.

### TypedKey Class

A frozen dataclass that provides type-safe entity identification with hierarchical path information.

```python
@dataclass(frozen=True)
class TypedKey:
    path: tuple[str, ...]  # Hierarchical path using instance IDs
    entity_type: EntityType
    entity_id: str  # The actual ID of the entity
```

#### Factory Methods

```python
# Type-safe factory methods
TypedKey.for_part(path, part_id)
TypedKey.for_rigid_assembly(path, assembly_id)
TypedKey.for_flexible_assembly(path, assembly_id)
TypedKey.for_feature(path, feature_id)
TypedKey.for_occurrence(path, occurrence_id)
```

#### Key Properties

- **Immutable**: Uses `@dataclass(frozen=True)` for hash stability
- **Hierarchical**: `path` tuple represents assembly hierarchy
- **Type-safe**: Entity type is part of the key structure
- **Debuggable**: Clear string representation showing type, path, and ID

#### Usage Examples

```python
# Create keys for different entity types
part_key = TypedKey.for_part(("assembly1", "subassembly1"), "part123")
rigid_key = TypedKey.for_rigid_assembly(("assembly1",), "subassembly123")
flexible_key = TypedKey.for_flexible_assembly(("assembly1",), "subassembly456")

# Access path information
print(part_key.depth)  # 2
print(part_key.parent_path)  # ("assembly1",)

# String representation for debugging
print(part_key)  # "PART:assembly1:subassembly1:part123"
```

## IdToNameResolver

Centralizes ID-to-name resolution with efficient single-pass construction.

### Key Features

- **Single-pass construction**: Builds all mappings in one traversal
- **Sanitized names**: Automatically applies name sanitization
- **Efficient lookups**: O(1) name resolution by ID
- **Instance tracking**: Maps both names and instance objects

### Usage

```python
resolver = IdToNameResolver(assembly)

# Get sanitized name for an instance ID
name = resolver.get_name(instance_id)

# Get the actual instance object
instance = resolver.get_instance(instance_id)

# Check if ID exists
if resolver.has_id(instance_id):
    # Process the instance
    pass
```

## KeyNamer Class

Provides type-aware naming with consistent output and reverse lookup capabilities.

### Core Features

- **Type-aware caching**: Separate reverse lookup caches by entity type
- **Prefix support**: Scoped caches for subassembly contexts
- **Clean naming**: Generates URDF-compatible names
- **Efficient lookups**: Fast name-to-key reverse resolution

### Naming Strategy

```python
key_namer = KeyNamer(id_resolver, joiner=":")

# Generate clean names from typed keys
name = key_namer.get_name(typed_key)          # Basic naming
prefixed_name = key_namer.get_name(typed_key, prefix="subasm")  # With prefix

# Reverse lookup (after building cache)
key_namer.build_reverse_cache(list_of_keys)
found_key = key_namer.lookup_key(name, EntityType.PART)
```

### Cache Management

The KeyNamer maintains separate caches for different entity types to avoid naming collisions:

```python
# Build reverse cache for efficient lookups
key_namer.build_reverse_cache(all_keys)

# Type-safe reverse lookups
part_key = key_namer.lookup_key("part_name", EntityType.PART)
rigid_key = key_namer.lookup_key("assembly_name", EntityType.RIGID_ASSEMBLY)

# Prefixed lookups for subassembly contexts
scoped_key = key_namer.lookup_key("local_name", EntityType.PART, prefix="subasm_prefix")
```

## Integration with CAD Pipeline

### Assembly Classification

The system automatically classifies assemblies based on `max_depth` parameter:

```python
# Determine rigidity at parse-time
if max_depth == 0:
    is_rigid = True  # All assemblies are rigid
else:
    is_rigid = instance_depth > max_depth

# Create appropriate typed key
if is_rigid:
    key = TypedKey.for_rigid_assembly(path, assembly_id)
    cad.rigid_assemblies[key] = instance
else:
    key = TypedKey.for_flexible_assembly(path, assembly_id)
    cad.flexible_assemblies[key] = instance
```

### Type-Safe Access Patterns

```python
# Access assemblies by type
for key, assembly in cad.rigid_assemblies.items():
    # Process rigid assemblies (single bodies)
    process_rigid_assembly(assembly)

for key, assembly in cad.flexible_assemblies.items():
    # Process flexible assemblies (articulated joints)
    process_flexible_assembly(assembly)

# Type-safe lookups
rigid_assembly = cad.lookup_rigid_assembly("assembly_name")
flexible_assembly = cad.lookup_flexible_assembly("assembly_name")
```

## Constants and Configuration

### Path Constants

```python
ROOT_PATH_NAME = "root"                    # For empty path display
SUBASSEMBLY_PATH_PREFIX = "subassembly"    # Subassembly path prefix
```

### Assembly Classification

```python
RIGID_ASSEMBLY_ONLY_FEATURE_TYPES = {AssemblyFeatureType.MATEGROUP}
```

### Joiners

```python
SUBASSEMBLY_JOINER = f"_{uuid.uuid4().hex[:4].upper()}_"  # Random unique separator
MATE_JOINER = f"_{uuid.uuid4().hex[:4].upper()}_"         # Random unique separator
```

## Best Practices

### 1. Use Type-Safe Factory Methods

```python
# Good: Type-safe and explicit
key = TypedKey.for_rigid_assembly(path, assembly_id)

# Avoid: Generic construction
key = TypedKey(path, EntityType.RIGID_ASSEMBLY, assembly_id)
```

### 2. Build Reverse Caches Strategically

```python
# Build cache once after populating all keys
all_keys = list(cad.parts.keys()) + list(cad.rigid_assemblies.keys()) + ...
cad.key_namer.build_reverse_cache(all_keys)

# Then use efficient lookups
part_key = cad.key_namer.lookup_key(name, EntityType.PART)
```

### 3. Leverage Type Information

```python
# Use entity type for processing decisions
if key.entity_type == EntityType.RIGID_ASSEMBLY:
    # Handle as single body
    process_as_single_body(assembly)
elif key.entity_type == EntityType.FLEXIBLE_ASSEMBLY:
    # Handle with articulated joints
    process_with_joints(assembly)
```

### 4. Handle Hierarchy Correctly

```python
# Access hierarchical information
if key.depth > max_assembly_depth:
    # Handle deeply nested assemblies
    flatten_deep_assembly(key)

# Navigate parent relationships
parent_path = key.parent_path
```

## Migration from Legacy Code

### Replace Boolean Checks

```python
# Old approach
if assembly_instance.isRigid:
    process_rigid_assembly()

# New approach
if key.entity_type == EntityType.RIGID_ASSEMBLY:
    process_rigid_assembly()
```

### Update Collection Access

```python
# Old approach
for key, assembly in cad.assemblies.items():
    if assembly.isRigid:
        # Handle rigid
    else:
        # Handle flexible

# New approach
for key, assembly in cad.rigid_assemblies.items():
    # Handle rigid assemblies

for key, assembly in cad.flexible_assemblies.items():
    # Handle flexible assemblies
```

### Use Type-Safe Lookups

```python
# Old approach
assembly = cad.lookup_assembly(name)
if assembly and assembly.isRigid:
    # Process

# New approach
rigid_assembly = cad.lookup_rigid_assembly(name)
if rigid_assembly:
    # Process rigid assembly
```

## Performance Benefits

1. **Compile-time Type Safety**: Eliminates runtime type checking overhead
2. **Efficient Collections**: Direct access to typed collections without filtering
3. **Optimized Caches**: Separate caches prevent unnecessary type checking during lookups
4. **Single-pass Construction**: All mappings built in one efficient traversal

## Future Extensions

The system is designed to be easily extensible:

```python
# Easy to add new entity types
class EntityType(Enum):
    # ... existing types
    CONSTRAINT = "CONSTRAINT"      # Future: constraint entities
    MATERIAL = "MATERIAL"          # Future: material assignments
    PATTERN = "PATTERN"            # Future: pattern entities

# Corresponding factory methods
TypedKey.for_constraint(path, constraint_id)
TypedKey.for_material(path, material_id)
TypedKey.for_pattern(path, pattern_id)
```
