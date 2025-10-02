# Handling Subassembly Mates in KinematicGraph

## Problem

Mates stored in subassembly registries use **relative PathKeys** (just the leaf IDs), but the KinematicGraph needs **absolute PathKeys** (full paths from root) to match instances in the global registries.

### Example

**Subassembly mate (relative):**

```python
# In subassembly McQ65EsxX+4zImFWp:
parent = PathKey(("MUgiNm7M17UkTi3/g",))          # Relative!
child = PathKey(("MemtmVozirMm3DOIv",))            # Relative!
```

**What we need (absolute):**

```python
parent = PathKey(("McQ65EsxX+4zImFWp", "MUgiNm7M17UkTi3/g"))     # Absolute
child = PathKey(("McQ65EsxX+4zImFWp", "MemtmVozirMm3DOIv"))      # Absolute
```

## Solution

The `KinematicGraph._collect_all_mates()` method now automatically converts subassembly mates to use absolute PathKeys.

### Implementation

```python
def _collect_all_mates(self) -> dict[tuple[PathKey, PathKey], MateFeatureData]:
    all_mates: dict[tuple[PathKey, PathKey], MateFeatureData] = {}

    # Root assembly mates already have absolute PathKeys
    all_mates.update(self.cad.root_assembly.mates.mates)

    # Convert subassembly mates from relative to absolute PathKeys
    for sub_key, assembly_data in self.cad.sub_assemblies.items():
        if len(assembly_data.mates.mates) > 0:
            absolute_mates = self._convert_subassembly_mates_to_absolute(
                assembly_data.mates.mates,
                sub_key  # Subassembly PathKey used as prefix
            )
            all_mates.update(absolute_mates)

    return all_mates
```

### PathKey Conversion

```python
def _make_absolute_pathkey(self, relative_key: PathKey, prefix_key: PathKey) -> PathKey:
    """Convert relative PathKey to absolute by prepending subassembly path."""
    # Check if already absolute (contains prefix)
    prefix_len = len(prefix_key.path)
    if len(relative_key.path) > prefix_len and relative_key.path[:prefix_len] == prefix_key.path:
        return relative_key

    # Prepend prefix to make absolute
    absolute_path = prefix_key.path + relative_key.path
    return PathKey(absolute_path)
```

### Example Conversion

**Before (from subassembly registry):**

```python
Subassembly: PathKey(("McQ65EsxX+4zImFWp",))
Mate:
  parent = PathKey(("MUgiNm7M17UkTi3/g",))
  child = PathKey(("MemtmVozirMm3DOIv",))
```

**After (in KinematicGraph):**

```python
Mate:
  parent = PathKey(("McQ65EsxX+4zImFWp", "MUgiNm7M17UkTi3/g"))
  child = PathKey(("McQ65EsxX+4zImFWp", "MemtmVozirMm3DOIv"))
```

## Why This Matters

### Without Conversion

- Subassembly mates have relative PathKeys
- These don't match any instances in global registries
- Mates get filtered as "invalid"
- Result: Disconnected graph with missing edges

### With Conversion

- All mates have absolute PathKeys
- PathKeys match instances in global registries
- All valid mates are included in graph
- Result: Fully connected kinematic graph

## Design Rationale

### Why Not Fix in parse.py?

We could have modified `CAD.process_mates_and_relations()` to consolidate all mates into the root registry with absolute PathKeys. However:

**Advantages of Current Approach:**

1. **Separation of concerns**: CAD class maintains assembly structure as-is
2. **Flexibility**: Different consumers can handle relative/absolute as needed
3. **Minimal changes**: KinematicGraph handles conversion locally
4. **Cleaner**: Subassembly data stays self-contained

**Why KinematicGraph Handles It:**

- KinematicGraph needs a global view of all mates anyway
- Conversion is straightforward (prefix concatenation)
- No need to modify original CAD data structures
- Easier to debug (conversion happens in one place)

### Alternative Considered

We could have modified `CAD._populate_subassembly()` to store mates with absolute PathKeys from the start:

```python
# NOT implemented - kept relative PathKeys
def _populate_subassembly(self, subassembly: SubAssembly, sub_key: PathKey):
    for mate in subassembly.features:
        # Convert to absolute PathKeys here?
        parent_abs = PathKey(sub_key.path + mate.parent_path)
        child_abs = PathKey(sub_key.path + mate.child_path)
        # ...
```

**Why we didn't:**

- Subassembly data would lose its self-contained nature
- Would need to track subassembly context everywhere
- Makes subassembly data harder to understand in isolation
- More invasive change to core data structures

## Testing

### Before Fix

```
[WARNING] Filtered out 4 mates with invalid PathKeys (3/7 mates remain)
[INFO] KinematicGraph created: 2 nodes, 1 edges, root=...
```

### After Fix

```
[INFO] KinematicGraph created: 7 nodes, 6 edges, root=M0cLO6yVimMv6KhRM
```

All mates from subassemblies now correctly contribute to the kinematic graph!

## Usage

No changes required from users - the conversion happens automatically:

```python
cad = CAD.from_assembly(assembly, max_depth=1)
cad.process_mates_and_relations()

# Mates are automatically converted to absolute PathKeys
tree = KinematicGraph(cad, use_user_defined_root=True)

print(tree)  # Shows all nodes and edges including subassembly mates
```

## Related Documentation

- [KinematicGraph Migration Guide](kinematic-tree-migration.md)
- [KinematicGraph Troubleshooting](kinematic-tree-troubleshooting.md)
- [Developing Documentation](developing.md)
