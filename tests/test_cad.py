"""Tests for the CAD flattening pipeline."""

from __future__ import annotations

from onshape_robotics_toolkit.models.assembly import AssemblyInstance, MateFeatureData, PartInstance
from onshape_robotics_toolkit.parse import CAD, PathKey


def test_cad_metadata_and_registry_counts(cad_doc: CAD) -> None:
    """Smoke test: ensure the flat registries are populated with the expected counts."""
    # These numbers come from the static assembly.json test fixture and help catch regressions.
    assert len(cad_doc.keys_by_id) == 12
    assert len(cad_doc.instances) == 12
    assert len(cad_doc.occurrences) == 12
    assert len(cad_doc.parts) == 9
    assert len(cad_doc.mates) == 8
    assert len(cad_doc.patterns) == 0


def test_pathkey_indexes_are_consistent(cad_doc: CAD) -> None:
    """keys_by_id and keys_by_name should contain the same PathKeys."""
    assert set(cad_doc.keys_by_id.values()) == set(cad_doc.keys_by_name.values())

    # Every registry should be keyed with canonical PathKeys from keys_by_id.
    valid_keys = set(cad_doc.keys_by_id.values())
    for registry in (cad_doc.instances, cad_doc.occurrences, cad_doc.parts):
        assert set(registry.keys()).issubset(valid_keys)


def test_instances_and_mates_use_pathkeys(cad_doc: CAD) -> None:
    """Instances are either PartInstance or AssemblyInstance indexed by PathKey."""
    for key, instance in cad_doc.instances.items():
        assert isinstance(key, PathKey)
        assert isinstance(instance, (PartInstance, AssemblyInstance))

    for (assembly_key, parent_key, child_key), mate in cad_doc.mates.items():
        assert assembly_key is None or isinstance(assembly_key, PathKey)
        assert isinstance(parent_key, PathKey)
        assert isinstance(child_key, PathKey)
        assert isinstance(mate, MateFeatureData)

        # Mated entities are normalized so entity[0] is always the parent.
        parent_occ = mate.matedEntities[0].matedOccurrence
        child_occ = mate.matedEntities[1].matedOccurrence

        assert parent_occ[-1] == parent_key.leaf
        assert child_occ[-1] == child_key.leaf

        # Subassembly mates carry full absolute paths, root mates remain relative.
        if len(parent_occ) == len(parent_key.path):
            assert parent_occ == list(parent_key.path)
        if len(child_occ) == len(child_key.path):
            assert child_occ == list(child_key.path)


def test_lookup_helpers_round_trip_pathkeys(cad_doc: CAD) -> None:
    """get_path_key and get_path_key_by_name should round-trip PathKeys."""
    sample_key = next(iter(cad_doc.instances.keys()))

    assert cad_doc.get_path_key(sample_key.path) == sample_key
    assert cad_doc.get_path_key_by_name(sample_key.name_path) == sample_key


def test_rigid_subassemblies_at_depth_one(cad_doc_depth_1: CAD) -> None:
    """When max_depth=1, nested assemblies become rigid and parts record their rigid parents."""
    rigid_instances = [
        key for key, inst in cad_doc_depth_1.instances.items() if isinstance(inst, AssemblyInstance) and inst.isRigid
    ]
    assert len(rigid_instances) == 2  # two second-level subassemblies become rigid

    remapped_parts = [
        (key, part.rigidAssemblyKey)
        for key, part in cad_doc_depth_1.parts.items()
        if getattr(part, "rigidAssemblyKey", None) is not None
    ]
    assert remapped_parts, "Expected parts inside rigid assemblies to record a rigidAssemblyKey"
    for _key, rigid_parent in remapped_parts:
        assert isinstance(rigid_parent, PathKey)
        assert rigid_parent in rigid_instances

    # Flexible mates are skipped once the parent assembly is rigid.
    assert len(cad_doc_depth_1.mates) == 6


def test_everything_rigid_at_depth_zero(cad_doc_depth_0: CAD) -> None:
    """At max_depth=0 the entire assembly collapses into rigid assemblies."""
    rigid_assemblies = [
        key for key, inst in cad_doc_depth_0.instances.items() if isinstance(inst, AssemblyInstance) and inst.isRigid
    ]
    assert len(rigid_assemblies) == 3  # root plus two nested occurrences

    rigid_parts = [part for part in cad_doc_depth_0.parts.values() if part.isRigidAssembly]
    assert len(rigid_parts) == 3  # synthetic Part entries for each rigid assembly

    # Only root-level mates survive because everything deeper is rigid.
    assert len(cad_doc_depth_0.mates) == 3
