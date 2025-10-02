# Test Suite Organization

This directory contains the test suite for the onshape-robotics-toolkit parsing system.

## Structure

```
tests/
├── conftest.py            # Shared fixtures, helpers, and configuration
├── test_pathkey.py        # Tests for PathKey class
├── test_cad_document.py   # Tests for CAD creation and population
├── test_lookups.py        # Tests for lookup functionality (instances, parts, occurrences)
├── test_subassemblies.py  # Tests for subassembly data structure and rigid assemblies
├── test_kinematic_tree.py # Tests for KinematicTree class
└── data/
    └── assembly.json      # Test assembly data
```

## Test Files

### `conftest.py`

Shared pytest configuration including:

- **Fixtures**: `assembly`, `cad_doc`, `cad_doc_depth_1`, `cad_doc_depth_2`, `cad_doc_all_depths`
- **Helper Functions**: Functions to extract parts, assemblies, occurrences from CAD
- **Constants**: `MAX_TEST_ASSEMBLY_DEPTH = 2`

### `test_pathkey.py`

Tests for the `PathKey` class:

- Creating PathKeys from tuples, lists, and strings
- Parent key relationships
- Immutability
- Leaf ID extraction

### `test_cad_document.py`

Tests for `CAD` class:

- Document creation from Assembly
- Registry population (instances, occurrences, mates, patterns)
- Parts dictionary population with PathKey indexing
- Assembly data structure documentation

### `test_lookups.py`

Tests for lookup functionality across the registries:

**`TestLookups` class:**

- Part instance lookups by PathKey
- Assembly instance lookups by PathKey
- Occurrence lookups by PathKey
- Part definition lookups by part ID
- Transform lookups
- Name-based lookups (with and without depth filtering)
- Hierarchical name generation
- Subassembly retrieval
- Rigid vs flexible assembly classification
- Parent key lookups

**`TestLookupsAllDepths` class:**

- Parametrized tests that run with `max_depth` values of 0, 1, and 2
- Verifies lookups work consistently across all depth configurations
- Tests rigid/flexible classification logic

### `test_subassemblies.py`

Tests for the separated subassembly data structure and rigid assembly handling:

**`TestSubassemblyDataStructure` class:**

- Subassemblies keyed by PathKey
- Each subassembly has its own AssemblyData with registries
- Mates separated by subassembly (not flattened into root)
- `get_all_mates()` aggregates correctly
- `get_subassembly_data()` retrieval

**`TestRigidAssemblyHandling` class:**

- `isRigid` flag set correctly based on depth
- `is_rigid_assembly()` uses the flag (not depth comparison)
- `rigid_count` and `flexible_count` properties
- Tests across max_depth=0, 1, and 2

**`TestRigidAssemblyAsPartObjects` class:**

- Rigid assemblies stored in `parts` dict
- Parts dict uses PathKey indexing
- Rigid assembly Part objects have `isRigidAssembly=True`
- Rigid assemblies have no partId but have elementId

**`TestInstanceStorageStrategy` class:**

- All instances in root registry (flat)
- Instances also in subassembly registries (hierarchical)
- Dual storage for efficient lookup and hierarchy preservation

### `test_kinematic_tree.py`

Tests for the `KinematicTree` class using PathKey-based system:

**`TestKinematicTree` class:**

- Creating kinematic tree from CAD document
- Graph has nodes (parts) and edges (mates)
- Root node detection (user-defined or centrality-based)
- Topological ordering calculation
- Node navigation (`get_children()`, `get_parent()`)
- Metadata access (`get_node_metadata()`)
- Mate data retrieval (`get_mate_data()`)
- Tree representation and visualization
- Tests across different max_depth values
- Verifies nodes are PathKeys
- Validates nodes match parts in CAD

**`TestKinematicTreeInternals` class:**

- Mate collection from root and subassemblies (`_collect_all_mates()`)
- Mate validation and filtering (`_validate_mates()`)
- Valid mate target checking (`_is_valid_mate_target()`)
- Parts involved in mates extraction (`_get_parts_involved_in_mates()`)
- User-defined root detection (`_find_user_defined_root()`)
- Relative to absolute PathKey conversion (`_make_absolute_pathkey()`)
- Idempotent PathKey conversion

**`TestRigidAssemblyMateHandling` class:**

- Rigid assembly ancestor detection (`_find_rigid_assembly_ancestor()`)
- Subassembly mate conversion to absolute PathKeys
- Internal mate filtering (within same rigid assembly)
- Cross-boundary mate remapping (to rigid assembly roots)

**`TestKinematicTreeVisualization` class:**

- `show()` method existence and functionality
- Label mapping creation (PathKey → part/assembly names)

**`TestKinematicTreeConnectivity` class:**

- Single connected component verification
- No self-loops in graph
- All nodes reachable from root

**Note:** These tests require assembly data at `data/assembly.json`. If the file is missing, tests will error during fixture setup.

## Parametrized Testing

The `cad_doc_all_depths` fixture automatically runs tests with three different `max_depth` configurations:

- `max_depth=0`: All assemblies are rigid
- `max_depth=1`: Depth-1 assemblies are flexible, deeper ones are rigid
- `max_depth=2`: All assemblies in our test data are flexible

Each test using `cad_doc_all_depths` runs **3 times** (once per depth value).

## Test Data

The test assembly (`data/assembly.json`) contains:

- 7 total parts
- 3 part instances (all at depth 1)
- 1 assembly instance (at depth 1)
- No nested parts within subassemblies

**Note**: Some tests may skip due to data limitations (e.g., no nested parts for parent lookup tests).

## Running Tests

```bash
# Run all tests
make test

# Run specific test file
pytest tests/test_lookups.py -v

# Run specific test class
pytest tests/test_lookups.py::TestLookupsAllDepths -v

# Run with verbose output
pytest tests/ -v -s

# Run with coverage
pytest tests/ --cov
```

## Adding New Tests

1. **Add shared fixtures** → `conftest.py`
2. **Add helper functions** → `conftest.py`
3. **Add test classes** → Appropriate test file based on functionality
4. Use parametrized fixtures (`cad_doc_all_depths`) when testing across different depth configurations
