# Coordinates Example - Assembly Structure

This example demonstrates mate remapping for rigid assemblies at `max_depth=1`.

## Onshape Document

**URL**: `https://cad.onshape.com/documents/1859bf4489c74b8d9d74e797/w/8e52be2776b88bd0b8524f80/e/46679c6ab890a1b7a6d11a88`

**Document**: Test assembly with nested subassemblies designed to test mate remapping logic.

## Assembly Hierarchy

```
Root Assembly (depth=0)
├─ Part 1 <1> (depth=0) - Fixed root part
├─ Part 14 <1> (depth=0) - External part
├─ Part 15 <1> (depth=0) - External part
└─ Assembly 2 <1> (depth=1) - FLEXIBLE subassembly
   ├─ Part 6 <1> (depth=1) - Part in flexible assembly
   ├─ Part 12 <1> (depth=1) - Part in flexible assembly
   ├─ Assembly 1 <1> (depth=2) - RIGID subassembly (depth > max_depth)
   │  ├─ Part 4 <1> (depth=2) - Buried in rigid assembly
   │  └─ Part 10 <1> (depth=2) - Buried in rigid assembly
   └─ Assembly 1 <2> (depth=2) - RIGID subassembly (pattern instance)
      ├─ Part 4 <1> (depth=2) - Buried in rigid assembly
      └─ Part 10 <1> (depth=2) - Buried in rigid assembly
```

## Key Characteristics

### At max_depth=1

- **Flexible Assemblies** (depth ≤ 1):
  - Root assembly (depth=0)
  - Assembly 2 (depth=1)

- **Rigid Assemblies** (depth > 1):
  - Assembly 1 <1> (depth=2)
  - Assembly 1 <2> (depth=2)

### Mate Structure (Before Remapping)

The mates from Assembly 2 (flexible) reference parts buried inside Assembly 1 (rigid):

1. **Revolute 1**: `Part 1 <1>` ↔ `Assembly 2 > Part 6 <1>` ✅ No remapping needed
2. **Revolute 2**: `Assembly 2 > Assembly 1 <2> > Part 10 <1>` ↔ `Part 14 <1>` ⚠️ Needs remapping
3. **Revolute 3**: `Assembly 2 > Assembly 1 <1> > Part 10 <1>` ↔ `Part 15 <1>` ⚠️ Needs remapping
4. **Revolute 1**: `Assembly 2 > Part 6 <1>` ↔ `Assembly 2 > Part 12 <1>` ✅ No remapping needed
5. **Revolute 2**: `Assembly 2 > Part 12 <1>` ↔ `Assembly 2 > Assembly 1 <1> > Part 4 <1>` ⚠️ Needs remapping
6. **Revolute 3**: `Assembly 2 > Part 12 <1>` ↔ `Assembly 2 > Assembly 1 <2> > Part 4 <1>` ⚠️ Needs remapping

### Expected Behavior After Remapping

The mate remapping system should:

1. **Detect buried parts**: Identify that `Part 4` and `Part 10` are inside rigid `Assembly 1`
2. **Remap PathKeys**: Change references from buried parts to rigid assembly root
   - `Assembly 2 > Assembly 1 <1> > Part 10` → `Assembly 2 > Assembly 1 <1>`
   - `Assembly 2 > Assembly 1 <2> > Part 10` → `Assembly 2 > Assembly 1 <2>`
   - `Assembly 2 > Assembly 1 <1> > Part 4` → `Assembly 2 > Assembly 1 <1>`
   - `Assembly 2 > Assembly 1 <2> > Part 4` → `Assembly 2 > Assembly 1 <2>`
3. **Update transforms**: Recompute `matedCS` to be relative to rigid assembly root
   - `T_rigid_mate = inv(T_world_rigid) @ T_world_part @ T_part_mate`
4. **Filter internal mates**: Remove any mates completely within a rigid assembly (none in this case)

### Final Graph Structure

After remapping, the kinematic graph should have:

- **7 nodes** (no disconnected components):
  - Part 1 (root, fixed)
  - Part 14
  - Part 15
  - Assembly 2 > Part 6
  - Assembly 2 > Part 12
  - Assembly 2 > Assembly 1 <1> (rigid assembly as single body)
  - Assembly 2 > Assembly 1 <2> (rigid assembly as single body)

- **6 edges** (all mates):
  - Part 1 ↔ Assembly 2 > Part 6
  - Assembly 2 > Assembly 1 <2> ↔ Part 14 (remapped)
  - Assembly 2 > Assembly 1 <1> ↔ Part 15 (remapped)
  - Assembly 2 > Part 6 ↔ Assembly 2 > Part 12
  - Assembly 2 > Part 12 ↔ Assembly 2 > Assembly 1 <1> (remapped)
  - Assembly 2 > Part 12 ↔ Assembly 2 > Assembly 1 <2> (remapped)

## Test Cases Covered

This assembly tests:

1. ✅ **Nested subassemblies**: Assembly 1 is nested inside Assembly 2
2. ✅ **Pattern instances**: Assembly 1 has two instances (<1> and <2>)
3. ✅ **Mate remapping**: Mates from flexible assembly reference parts in rigid assembly
4. ✅ **Transform computation**: Correctly computes relative transforms using world transforms
5. ✅ **No disconnected graphs**: All nodes should be connected after remapping

## Expected Output

```
INFO: Created CAD(keys=12, instances=12 (parts=9, asm=3), occurrences=12, mates=6, patterns=0, parts=11)
INFO: KinematicGraph created: 7 nodes, 6 edges, root=Part_1_1
```

**No warnings** about disconnected graphs or missing instances.

## Running the Example

```bash
# Activate virtualenv
source .venv/bin/activate

# Run from the coordinates directory
cd examples/coordinates
python main.py
```

## Debugging Tips

If you see disconnected graphs:

1. **Check mate remapping logs**: Look for "Remapping parent/child from X to rigid root Y" messages (debug level)
2. **Verify rigid marking**: Ensure `_mark_rigid_assemblies()` runs before `_populate_mates()`
3. **Inspect transforms**: Verify world transforms are available for both rigid root and buried part
4. **Check parts registry**: Rigid assemblies should be in `cad.parts` with `isRigidAssembly=True`

## Assembly Details

### Root-Level Instances (depth=0)

- **Part 1 <1>**: Fixed root part (base)
- **Part 14 <1>**: External part connected to Assembly 1 <2>
- **Part 15 <1>**: External part connected to Assembly 1 <1>
- **Assembly 2 <1>**: Main flexible subassembly

### Assembly 2 Contents (depth=1)

- **Part 6 <1>**: Connected to root (Part 1)
- **Part 12 <1>**: Connected to Part 6 and both Assembly 1 instances
- **Assembly 1 <1>**: First instance of rigid subassembly
- **Assembly 1 <2>**: Second instance of rigid subassembly (pattern)

### Assembly 1 Contents (depth=2, RIGID)

Each Assembly 1 instance contains:
- **Part 4 <1>**: Interior part
- **Part 10 <1>**: Interior part

These parts are NOT individually accessible at `max_depth=1` - the entire Assembly 1 is treated as a single rigid body.

## Implementation Notes

- **Subassembly UIDs**: Both `Assembly 1 <1>` and `Assembly 1 <2>` share the same UID (`3f42b975226315d3`) because they're instances of the same subassembly definition
- **Element ID**: All instances point to the same `elementId` (`385e9f7e25af7f09ad89eaf6`)
- **Nested mate processing**: The mate processing correctly handles subassemblies at any depth by searching the flat `instances` registry
- **Transform availability**: All occurrence transforms are available regardless of `max_depth` for transform calculations
