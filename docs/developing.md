# Developer Handbook

This library turns an Onshape assembly into a robotic model that can be exported as URDF or MuJoCo MJCF. The goal of the toolkit is to make every stage of that conversion explicit and easy to extend. This document explains the current code structure and highlights the extension points you will touch when contributing.

## Architecture Overview

- **parse** (`onshape_robotics_toolkit/parse.py`): Flattens Onshape's assembly JSON into `PathKey` indexed registries on the `CAD` object. Handles rigid vs. flexible subassemblies, mate data, and pattern expansion.
- **graph** (`onshape_robotics_toolkit/graph.py`): Converts the `CAD` registries into a directed `KinematicGraph` where nodes are parts and edges are mates. Deals with rigid-assembly remapping, root selection, and graph cleanup.
- **robot** (`onshape_robotics_toolkit/robot.py`): Walks the `KinematicGraph` to build a `Robot` (an `nx.DiGraph`) populated with URDF/MJCF link and joint objects plus STL assets.

The pipeline is intentionally linear:

```
Assembly JSON → CAD (parse) → KinematicGraph (graph) → Robot (robot) → URDF/MJCF + assets
```

## End-to-End Pipeline

1. Use `Client` to fetch assembly data.
2. Call `CAD.from_assembly(assembly, max_depth, client)` to flatten the JSON.
3. Build a kinematic graph with `KinematicGraph.from_cad(cad, use_user_defined_root=True)`.
4. Generate the exportable robot via `Robot.from_graph(graph, client, name, robot_type)`.

```python
from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.robot import Robot

client = Client()
assembly = client.get_assembly(url)

cad = CAD.from_assembly(assembly, max_depth=1, client=client)
graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
robot = Robot.from_graph(graph, client=client, name="demo_bot")
robot.save("demo_bot.urdf", download_assets=True)
```

Passing a `Client` into `CAD.from_assembly` is optional in general, but required when `max_depth` forces subassemblies to become rigid—rigid assemblies need extra API calls to recover their internal transforms and mass properties.

## `parse.py` — Flattening Onshape Data

### PathKey fundamentals

- `PathKey` is a frozen dataclass that records both the raw instance ID path (`_path`) and a sanitized name path (`_name_path`).
- It preserves hierarchy (depth starts at 0 for root-level instances) and provides helpers like `.parent`, `.root`, `.name`, and ordering comparisons for consistent sorting.
- Every registry (`instances`, `occurrences`, `parts`, etc.) in `CAD` is keyed by `PathKey`.

### CAD registries

The `CAD` class stores a denormalized, dictionary-based view of an assembly:

- `keys_by_id` / `keys_by_name`: canonical lookups from ID or name tuples to `PathKey`.
- `instances`: every `PartInstance` and `AssemblyInstance` reachable from the root assembly, including nested occurrences.
- `occurrences`: absolute transforms for each instance, taken from the root assembly `occurrences` list.
- `subassemblies`: every `SubAssembly` definition copied to each placement (keyed by `PathKey`) with rigidity flags applied.
- `mates`: dictionary keyed by `(assembly_key, parent_key, child_key)` storing `MateFeatureData`. `assembly_key` is `None` for root-level mates and a `PathKey` for the owning subassembly.
- `patterns`: `Pattern` objects keyed by pattern id with seed/instance paths rewritten to absolute coordinates.
- `parts`: `Part` definitions (including synthetic parts for rigid assemblies) keyed by `PathKey`. Mass properties are fetched lazily.

### `CAD.from_assembly` ingestion order

`CAD.from_assembly` orchestrates several private populators. Order matters because later steps depend on data from earlier ones.

1. **Instance naming (`_build_id_to_name_map`)**: Builds a UID→name map from root and subassemblies before any `PathKey` creation.
2. **PathKey creation (`_build_path_keys_from_occurrences`)**: Iterates over root `occurrences` once to create all `PathKey` instances and seed the lookup dictionaries.
3. **Instances (`_populate_instances`)**: Recursively walks root `instances` and nested `SubAssembly.instances`, cloning each into the flat `instances` dict.
4. **Occurrences (`_populate_occurrences`)**: Stores transforms for every absolute occurrence in the root assembly list.
5. **Subassemblies (`_populate_subassemblies`)**: Copies each `SubAssembly` definition to every placement. If a placement depth is ≥ `max_depth`, the subassembly (and its corresponding `AssemblyInstance`) is marked rigid.
6. **Parts (`_populate_parts`)**:
   - Matches `PartInstance.uid` values back to part definitions and writes entries into `parts`.
   - Sets `worldToPartTF` from the current occurrence transform.
   - For parts buried inside rigid assemblies, records `rigidAssemblyKey`, `rigidAssemblyWorkspaceId`, and, if available, `rigidAssemblyToPartTF`. When the transform is missing, `fetch_occurrences_for_subassemblies` is invoked to retrieve `RootOccurrences` via the API.
   - Creates _synthetic_ `Part` objects for every rigid assembly placement so graph/robot stages can treat rigid assemblies like single parts.
7. **Mates (`_populate_mates`)**:
   - Walks root `features` plus every flexible subassembly’s `features`.
   - Writes mates using absolute `PathKey` pairs while preserving assembly provenance (the first tuple slot).
   - Normalizes `MateFeatureData.matedEntities` so index `0` is always the graph parent and index `1` the child.
8. **Patterns (`_populate_patterns`)**:
   - Rewrites `seedToPatternInstances` paths to absolute coordinates.
   - Calls `_flatten_patterns` to clone mates for every pattern instance. Cloned mates get transformed `MatedCS` values so pattern copies behave like unique joints.

Populating mates after parts guarantees that pattern expansion and rigid-assembly remapping have the data they need. Patterns run last because they depend on both mates and occurrences.

### Rigid assemblies and `max_depth`

- `max_depth` is applied during `_populate_subassemblies`: placements at or deeper than the limit are marked rigid. Their mates are excluded from flexible processing, and their internal parts are remapped later.
- `get_rigid_assembly_root` walks up a `PathKey` hierarchy to find the top-most rigid assembly. The result is stored on `Part.rigidAssemblyKey`.
- `rigidAssemblyToPartTF` holds the transform from the rigid assembly origin to the buried part. When it is unavailable, `fetch_occurrences_for_subassemblies` uses `Client.get_root_assembly` to retrieve the subassembly’s own occurrences and fill in the missing data.
- Mass properties for rigid assemblies are fetched with `Client.get_assembly_mass_properties`, while regular parts use `Client.get_mass_property`.

### Asynchronous helpers

- `fetch_mass_properties_for_parts(client)` runs after graph creation (see `Robot.from_graph`) and only fetches data for parts whose `MassProperty` is still `None` and that are not remapped rigid subassembly members.
- `fetch_occurrences_for_subassemblies(client)` populates `SubAssembly.RootOccurrences` for rigid placements so remapping and mass properties stay correct.

### Lookup utilities

`CAD` provides several helpers for downstream consumers:

- `get_path_key(path)`: Convert an ID or path list/tuple into the canonical `PathKey`.
- `get_transform(path_key, wrt=None)`: Retrieve occurrence transforms with optional relative frame conversion.
- `get_mates_from_root`, `get_mates_from_subassembly`, `get_all_mates_flattened`, `get_mate_data`, `get_mate_assembly`: Query mates with or without provenance.

Use these helpers instead of touching the internal dictionaries—doing so keeps remapping and provenance logic centralized.

## `graph.py` — Building the kinematic graph

`KinematicGraph` extends `nx.DiGraph` and holds a directed representation of the robot’s mating structure. Construction is done via `KinematicGraph.from_cad(cad, use_user_defined_root=True)`.

### Build pipeline

1. **Mate remapping (`_remap_mates`)**: Before any graph logic, mates are rewritten so parts inside rigid assemblies are replaced with the rigid assembly’s synthetic part. The method updates both `MateFeatureData` and `matedEntities[*].matedOccurrence` and adjusts `MatedCS` values using `rigidAssemblyToPartTF`.
2. **Determine involved parts (`_get_parts_involved_in_mates`)**: Collects every part that appears in a mate. This is the node set for the undirected graph.
3. **Initial graph (`create_graph`)**: Builds an undirected `networkx.Graph` so connected-component and root detection work with symmetric edges. Every node stores only the `PathKey`; node attributes are added later.
4. **Graph processing (`_process_graph`)**:
   - `remove_disconnected_subgraphs` trims the graph down to the largest connected component and prints a tree summary in the logs.
   - `_find_root_node` respects Onshape “fixed” occurrences if `use_user_defined_root` is `True`; otherwise it falls back to closeness centrality.
   - A BFS tree from the root is used to orient the graph. Nodes are added with their full `Part` objects (`data=part`) so downstream stages have access to metadata.
   - Edges inherit `MateFeatureData`. If the BFS orientation disagrees with the parent/child order captured earlier, `_process_graph` reverses `matedEntities` so downstream code always sees parent→child ordering.
   - Loops or extra edges not in the BFS tree are reattached using their stored orientation.

### Node and edge payloads

- **Nodes**: keyed by `PathKey`, with attributes `data=<Part>`.
- **Edges**: parent→child pairs with attribute `data=<MateFeatureData>`.
- The `KinematicGraph.root` attribute stores the root `PathKey`. `topological_order` is currently implicit (iterate over `nx.bfs_tree(graph, graph.root)` to reproduce the robot build order).

### Utilities

- `convert_to_digraph`, `remove_disconnected_subgraphs`, `create_graph`, and `show()` are exposed for experimentation/debugging.
- `show()` plots the graph with sanitized names. Use it when debugging connectivity issues.
- Because the graph mutates copies of mate data, upstream registries in `CAD` remain untouched.

## `robot.py` — Generating robot models

`Robot` subclasses `nx.DiGraph` and ultimately holds the URDF/MJCF-ready structure.

### Creation (`Robot.from_graph`)

1. Optionally fetch mass properties by calling `asyncio.run(kinematic_graph.cad.fetch_mass_properties_for_parts(client))`.
2. Instantiate `Robot`, preserving the original `KinematicGraph` reference for later inspection.
3. Add the root link using the root node’s `Part` data.
4. Traverse every edge in the graph:
   - Retrieve `MateFeatureData` from the edge.
   - Call `get_robot_joint` to convert the mate into URDF joints (fastened → `FixedJoint`, revolute → `RevoluteJoint`, slider/cylindrical → `PrismaticJoint`, ball → three chained revolute joints with dummy links).
   - Call `get_robot_link` to create the child `Link`, compute its transform, and prepare an `Asset` descriptor.
   - Add the child link (and any dummy links) as nodes and register the joint(s) as edges on the robot.

Nodes carry three pieces of data:

- `data`: the URDF/MJCF `Link`.
- `asset`: an `Asset` descriptor, or `None` for dummy links.
- `world_to_link_tf`: cached homogeneous transform for later reuse.

Edges carry `data=<BaseJoint>` instances.

### Link generation (`get_robot_link`)

- Starts with the child mate coordinate system when available, falling back to `Part.worldToPartTF`.
- Computes mass, inertia, and center of mass if `MassProperty` exists; otherwise defaults are logged.
- Determines how to fetch STL assets:
  - Regular parts use `WorkspaceType.M` (microversion) and `part.documentMicroversion`.
  - Rigid assemblies use `WorkspaceType.W` with `rigidAssemblyWorkspaceId`.
  - Versioned parts use `WorkspaceType.V` and `documentVersion`.
- Produces a `Link` with matching `VisualLink` and `CollisionLink`. Materials are randomly assigned for visualization.

### Joint generation (`get_robot_joint`)

- Respect the normalized parent/child order established in the graph.
- Creates `Origin` from the parent part frame to mate frame transform.
- Maintains a `used_joint_names` set to ensure URDF-safe unique joint names.
- Handles mimic joints, dummy links for ball mates, and keeps placeholders for future dynamics/limits enhancements.

### Export and utilities

- `save(path, download_assets=True)` writes URDF/MJCF XML and optionally downloads STL assets through the `Asset` objects.
- `to_urdf` and `to_mjcf` generate XML trees.
- `show_tree` and `show_graph` visualize the resulting robot structure for debugging.

## Working With the Onshape API

- `Client` centralizes all API calls: authentication, assembly fetch, mass properties, and STL downloads.
- All network work happens via `asyncio.to_thread` to avoid blocking the main thread. If you add new API interactions, mirror this approach so we stay thread-safe without rewriting the pipeline as fully async.
- Keep `WorkspaceType` selection accurate—using a microversion when a workspace is required will trigger 404/409 responses from Onshape.

## Debugging Tips

- Inspect `CAD` state quickly with `repr(cad)`; it prints counts for every registry.
- Use `cad.mates.items()` to confirm mate orientation and provenance before the graph stage.
- Call `graph.show()` or `robot.show_graph()` when debugging connectivity issues.
- When rigid assemblies behave oddly, confirm `rigidAssemblyToPartTF` is set. If not, ensure `CAD.from_assembly` received a `Client` so it can fetch `RootOccurrences`.

## Testing

The test suite validates critical functionality across the entire pipeline with **52 tests** providing **48% coverage** of core logic. Tests are designed to run quickly (<1 second) without requiring Onshape API access.

### Test Structure

Tests are organized by functionality in the `tests/` directory:

- **`test_urdf_generation.py`** (5 tests): End-to-end URDF generation with golden file comparison
- **`test_transforms.py`** (16 tests): Coordinate frame transformations (MatedCS, Origin, joint/link positioning)
- **`test_robot.py`** (6 tests): Robot generation, mate type coverage, joint limits, naming
- **`test_kinematic_graph.py`** (9 tests): Graph construction, validation, rigid remapping
- **`test_cad.py`** (11 tests): CAD parsing, rigid subassembly handling, name sanitization
- **`test_pathkey.py`** (5 tests): PathKey behavior, sorting, validation

### Running Tests

```bash
# Run all tests
pytest tests/ -v

# Run specific test module
pytest tests/test_transforms.py -v

# Run with coverage report
pytest --cov --cov-report=term-missing

# Run single test
pytest tests/test_transforms.py::TestMatedCSTransformations::test_identity_transform -v
```

### Testing Approach

**1. Golden File Testing** (`test_urdf_generation.py`)

Tests compare generated URDF output against known-good reference files:

```python
# tests/data/assembly_expected.urdf is the golden file
urdf_output = robot.to_urdf()
is_equal, differences = compare_urdf_files(
    generated_urdf, expected_urdf,
    tolerance=1e-6, ignore_colors=True
)
assert is_equal, f"URDF differs: {differences}"
```

This catches regressions in URDF structure, transforms, or joint/link generation.

**2. Transform Validation** (`test_transforms.py`)

Tests validate coordinate frame transformations at multiple levels:

- **MatedCS transformations**: Identity, translation, rotation, composition
- **Origin calculations**: Matrix → Euler angle extraction
- **Joint origins**: Parent frame composition with `world_to_parent_tf`
- **Ball joints**: 3-DOF decomposition into revolute joints + dummy links

All comparisons use `np.allclose()` with tolerance for floating-point stability.

**3. Mate Type Coverage** (`test_robot.py`)

Every supported mate type is tested:

```python
# REVOLUTE → RevoluteJoint with axis
# FASTENED → FixedJoint
# SLIDER/CYLINDRICAL → PrismaticJoint
# BALL → 3 RevoluteJoints + 2 dummy links
# PLANAR → DummyJoint (unsupported)
```

Tests verify correct joint type, axis direction, and limit values.

**4. Rigid Subassembly Testing** (`test_cad.py`, `test_kinematic_graph.py`)

Tests validate the complex rigid subassembly remapping logic:

- `rigidAssemblyKey` assignment for parts within rigid assemblies
- `rigidAssemblyToPartTF` transform propagation
- Mate filtering (internal mates removed, external mates preserved)
- Graph node depth limits when `max_depth` is applied

**5. Mocking External Dependencies**

Tests use mock clients to avoid network calls:

```python
@dataclass
class DummyClient:
    def download_part_stl(self, *_, **__):
        raise RuntimeError("No network calls in unit tests")
```

This keeps tests fast and deterministic.

### Test Fixtures

Shared fixtures provide consistent test data:

- **`assembly_json_path`**: Path to `tests/data/assembly.json`
- **`assembly`**: Loaded Assembly object
- **`cad_doc`**: CAD with `max_depth=2` (all flexible)
- **`cad_doc_depth_1`**: CAD with `max_depth=1` (nested assemblies rigid)
- **`cad_doc_depth_0`**: CAD with `max_depth=0` (all assemblies rigid)

These fixtures test the same assembly at different rigidity levels.

### Adding New Tests

When contributing new features:

1. **Add tests in the appropriate module**:

   - Transform logic → `test_transforms.py`
   - New mate type → `test_robot.py`
   - Parsing changes → `test_cad.py`
   - Graph modifications → `test_kinematic_graph.py`

2. **Use parametrized tests** for multiple configurations:

   ```python
   @pytest.mark.parametrize("mate_type,expected_joint", [
       (MateType.REVOLUTE, RevoluteJoint),
       (MateType.SLIDER, PrismaticJoint),
   ])
   def test_mate_conversion(mate_type, expected_joint):
       ...
   ```

3. **Update golden files** when URDF output changes intentionally:

   ```bash
   # Regenerate expected output
   python -c "from tests.conftest import ...; generate_expected_urdf()"
   ```

4. **Test edge cases**: Gimbal lock, name conflicts, disconnected graphs, etc.

### Coverage Goals

Current coverage focuses on core logic:

- **models/assembly.py**: 89% (mate handling, transforms)
- **parse.py**: 63% (CAD construction, rigid remapping)
- **graph.py**: 61% (graph building, validation)
- **models/link.py**: 40% (link generation)
- **models/joint.py**: 40% (joint types)

Areas needing more coverage:

- **connect.py**: 22% (API client - mostly needs integration tests)
- **robot.py**: 35% (MJCF export, asset download)
- **utilities/helpers.py**: 37% (utility functions)

## Contribution Checklist

- Understand which stage you are modifying:
  1. **parse** for ingesting or transforming Onshape data.
  2. **graph** for reasoning about connectivity or kinematics.
  3. **robot** for export formats, joint/link behavior, or asset management.
- Preserve invariants:
  - `CAD.mates` must always store parent→child ordering.
  - Graph nodes/edges should only contain deep copies of data (no in-place mutations of `CAD` registries).
  - Robot node keys remain `PathKey` objects so we can trace back to CAD data.
- **Add tests alongside new features**. Focus on:
  - PathKey handling (depth/order) when touching the parser.
  - Graph connectivity/root selection when altering graph logic.
  - Joint/link outputs when introducing new mate types.
  - Transform correctness using `np.allclose()` comparisons.
  - Golden file updates for URDF/MJCF changes.
- Run the full test suite before committing:
  ```bash
  pytest tests/ -v
  make check  # Runs linting, type checking, and tests
  ```
- Document new behavior here and keep inline comments concise. If you introduce a new pipeline stage or helper, summarize it in this handbook so future contributors know where to look.

Keeping this document aligned with the code makes onboarding new contributors faster and protects the assumptions baked into each stage of the pipeline. Update it anytime you change the parse/graph/robot trio or introduce new developer-facing workflows.
