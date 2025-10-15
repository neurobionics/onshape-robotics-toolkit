"""Tests for the CAD flattening pipeline."""

from __future__ import annotations

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.models.assembly import Assembly, AssemblyInstance, MateFeatureData, PartInstance
from onshape_robotics_toolkit.parse import CAD, PathKey


def test_cad_from_url(assembly: Assembly) -> None:
    test_url = (
        "https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/"
        "w/0d17b8ebb2a4c76be9fff3c7/e/d8f8f1d9dbf9634a39aa7f5b"
    )

    called: dict[str, object] = {}

    class FakeClient:
        def get_assembly(
            self,
            did: str,
            wtype: str,
            wid: str,
            eid: str,
            configuration: str = "default",
            log_response: bool = True,
            with_meta_data: bool = True,
        ) -> Assembly:
            called["args"] = (did, wtype, wid, eid, configuration, log_response, with_meta_data)
            return assembly

    client = FakeClient()

    cad = CAD.from_url(
        test_url,
        client=client,  # type: ignore[arg-type]
        max_depth=2,
        configuration="custom",
        log_response=False,
        with_meta_data=False,
    )

    assert isinstance(cad, CAD)
    assert called["args"] == (
        "a1c1addf75444f54b504f25c",
        "w",
        "0d17b8ebb2a4c76be9fff3c7",
        "d8f8f1d9dbf9634a39aa7f5b",
        "custom",
        False,
        False,
    )


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


def test_rigid_subassembly_key_assignment(cad_doc_depth_1: CAD) -> None:
    """Parts within rigid subassemblies should have rigidAssemblyKey assigned."""
    # Find all parts that are within rigid assemblies
    parts_with_rigid_parent = [
        (key, part)
        for key, part in cad_doc_depth_1.parts.items()
        if hasattr(part, "rigidAssemblyKey") and part.rigidAssemblyKey is not None
    ]

    assert len(parts_with_rigid_parent) > 0, "Expected some parts to have rigid assembly parents"

    for _key, part in parts_with_rigid_parent:
        # rigidAssemblyKey should point to a valid rigid assembly instance
        assert part.rigidAssemblyKey in cad_doc_depth_1.instances
        rigid_instance = cad_doc_depth_1.instances[part.rigidAssemblyKey]
        assert isinstance(rigid_instance, AssemblyInstance)
        assert rigid_instance.isRigid


def test_rigid_subassembly_transform_propagation(cad_doc_depth_1: CAD) -> None:
    """rigidAssemblyToPartTF should be set for parts within rigid assemblies when available."""
    parts_in_rigid = [
        part
        for part in cad_doc_depth_1.parts.values()
        if hasattr(part, "rigidAssemblyToPartTF") and part.rigidAssemblyToPartTF is not None
    ]

    # Note: This may be 0 if the Client is not provided or root occurrences are not fetched
    # In that case, we just verify the structure when transforms ARE present
    if len(parts_in_rigid) > 0:
        for part in parts_in_rigid:
            # Transform should be a valid MatedCS
            from onshape_robotics_toolkit.models.assembly import MatedCS

            assert isinstance(part.rigidAssemblyToPartTF, MatedCS)

            # Transform should be a valid 4x4 matrix
            tf = part.rigidAssemblyToPartTF.to_tf
            import numpy as np

            assert tf.shape == (4, 4)
            # Last row should be [0, 0, 0, 1]
            assert np.allclose(tf[3, :], [0, 0, 0, 1])
    else:
        # If no transforms are set, at least verify the rigid assembly keys are set
        parts_with_rigid_key = [
            part
            for part in cad_doc_depth_1.parts.values()
            if hasattr(part, "rigidAssemblyKey") and part.rigidAssemblyKey is not None
        ]
        # There should be some parts with rigidAssemblyKey even if transforms aren't set
        assert len(parts_with_rigid_key) > 0, "Expected some parts to have rigid assembly keys"


def test_sanitized_names_are_unique(cad_doc: CAD) -> None:
    """All PathKey string representations should be unique."""
    names = [str(key) for key in cad_doc.instances]
    assert len(names) == len(set(names)), "PathKey names should be unique"


def test_mate_remapping_for_rigid_subassemblies(cad_doc_depth_1: CAD) -> None:
    """Mates involving rigid subassemblies should be properly remapped."""
    # At depth 1, some nested assemblies become rigid
    # Their internal mates should be filtered out
    # Only mates connecting to/from rigid assemblies should remain

    for (_asm_key, parent_key, child_key), _mate in cad_doc_depth_1.mates.items():
        # Parent and child should be valid keys in the parts registry
        assert parent_key in cad_doc_depth_1.parts, f"Parent {parent_key} not in parts"
        assert child_key in cad_doc_depth_1.parts, f"Child {child_key} not in parts"

        # If a mate involves a rigid assembly part, it should not be an internal mate
        parent_part = cad_doc_depth_1.parts[parent_key]
        child_part = cad_doc_depth_1.parts[child_key]

        # Neither should be a "child part within a rigid assembly"
        # (i.e., parts with rigidAssemblyToPartTF set are internal and shouldn't have mates)
        assert parent_part.rigidAssemblyToPartTF is None, "Mate parent should not be internal to rigid assembly"
        assert child_part.rigidAssemblyToPartTF is None, "Mate child should not be internal to rigid assembly"


def test_client_api_call_counter_initialization() -> None:
    """Test that Client initializes with API call counter at 0."""
    client = Client(env="tests/test.env")
    assert client.api_call_count == 0


def test_client_reset_api_call_count() -> None:
    """Test that reset_api_call_count() resets the counter to 0."""
    client = Client(env="tests/test.env")
    # Manually increment counter
    client._api_call_count = 5
    assert client.api_call_count == 5

    # Reset counter
    client.reset_api_call_count()
    assert client.api_call_count == 0


def test_cad_estimate_api_calls(cad_doc: CAD) -> None:
    """Test CAD instance method for API call estimation."""
    estimation = cad_doc.estimate_api_calls(fetch_mass_properties=True, download_meshes=True)

    # Verify structure of returned dictionary (no 'base' anymore)
    assert "subassemblies" in estimation
    assert "mass_properties" in estimation
    assert "meshes" in estimation
    assert "total" in estimation

    # All values should be non-negative integers
    for value in estimation.values():
        assert isinstance(value, int)
        assert value >= 0

    # Total should be sum of components
    assert estimation["total"] == (estimation["subassemblies"] + estimation["mass_properties"] + estimation["meshes"])


def test_cad_estimate_api_calls_with_rigid_subassemblies(cad_doc_depth_1: CAD) -> None:
    """Test API call estimation with rigid subassemblies at max_depth=1."""
    estimation = cad_doc_depth_1.estimate_api_calls(fetch_mass_properties=True, download_meshes=True)

    # At depth 1, there should be rigid subassemblies
    assert estimation["subassemblies"] > 0, "Expected some rigid subassemblies at max_depth=1"

    # Total should include subassembly calls
    assert estimation["total"] > 0


def test_cad_estimate_api_calls_no_mass_properties(cad_doc: CAD) -> None:
    """Test CAD estimation without mass properties."""
    estimation = cad_doc.estimate_api_calls(fetch_mass_properties=False, download_meshes=True)

    # Mass properties should be 0
    assert estimation["mass_properties"] == 0

    # Meshes should still be counted
    assert estimation["meshes"] > 0


def test_cad_estimate_api_calls_no_meshes(cad_doc: CAD) -> None:
    """Test CAD estimation without mesh downloads."""
    estimation = cad_doc.estimate_api_calls(fetch_mass_properties=True, download_meshes=False)

    # Mass properties should be counted
    assert estimation["mass_properties"] > 0

    # Meshes should be 0
    assert estimation["meshes"] == 0
