"""Shared pytest fixtures and helpers for all tests."""

from pathlib import Path
from typing import Optional

import pytest

from onshape_robotics_toolkit.models.assembly import Assembly
from onshape_robotics_toolkit.parse import CADDocument, PathKey
from onshape_robotics_toolkit.utilities import load_model_from_json

# ============================================================================
# Constants
# ============================================================================

# Maximum depth for this test assembly (based on its structure)
MAX_TEST_ASSEMBLY_DEPTH = 2


# ============================================================================
# Fixtures - Data Loading
# ============================================================================


@pytest.fixture
def assembly_json_path() -> Path:
    """Path to the test assembly JSON file."""
    return Path(__file__).parent / "data" / "assembly.json"


@pytest.fixture
def assembly(assembly_json_path: Path) -> Assembly:
    """Load assembly from JSON file."""
    return load_model_from_json(Assembly, str(assembly_json_path))


# ============================================================================
# Fixtures - CADDocument at Various Depths
# ============================================================================


@pytest.fixture(params=[0, 1, 2])
def cad_doc_all_depths(assembly: Assembly, request) -> CADDocument:
    """Create CADDocument with all possible max_depth values (0, 1, 2).

    This parametrized fixture will run tests with each depth automatically.
    """
    max_depth = request.param
    return CADDocument.from_assembly(assembly, max_depth=max_depth)


@pytest.fixture
def cad_doc(assembly: Assembly) -> CADDocument:
    """Create CADDocument from assembly with max_depth=0 (all rigid)."""
    return CADDocument.from_assembly(assembly, max_depth=0)


@pytest.fixture
def cad_doc_depth_1(assembly: Assembly) -> CADDocument:
    """Create CADDocument from assembly with max_depth=1."""
    return CADDocument.from_assembly(assembly, max_depth=1)


@pytest.fixture
def cad_doc_depth_2(assembly: Assembly) -> CADDocument:
    """Create CADDocument from assembly with max_depth=2 (all flexible)."""
    return CADDocument.from_assembly(assembly, max_depth=2)


# ============================================================================
# Helper Functions - Data Extraction
# ============================================================================


def get_first_part_key(cad_doc: CADDocument) -> Optional[PathKey]:
    """Get the first part instance key, or None if no parts exist."""
    part_keys = list(cad_doc.root_assembly.instances.parts.keys())
    return part_keys[0] if part_keys else None


def get_first_assembly_key(cad_doc: CADDocument) -> Optional[PathKey]:
    """Get the first assembly instance key, or None if no assemblies exist."""
    assembly_keys = list(cad_doc.root_assembly.instances.assemblies.keys())
    return assembly_keys[0] if assembly_keys else None


def get_first_occurrence_key(cad_doc: CADDocument) -> Optional[PathKey]:
    """Get the first occurrence key, or None if no occurrences exist."""
    occurrence_keys = list(cad_doc.root_assembly.occurrences.occurrences.keys())
    return occurrence_keys[0] if occurrence_keys else None


def get_first_part_id(cad_doc: CADDocument) -> Optional[str]:
    """Get the first part ID, or None if no parts exist."""
    part_ids = list(cad_doc.parts.keys())
    return part_ids[0] if part_ids else None


def get_nested_part_key(cad_doc: CADDocument, min_depth: int = 2) -> Optional[PathKey]:
    """Get a nested part instance key (depth >= min_depth), or None if none exist."""
    nested_parts = {k: v for k, v in cad_doc.root_assembly.instances.parts.items() if k.depth >= min_depth}
    if not nested_parts:
        return None
    return next(iter(nested_parts.keys()))


def get_root_part_key(cad_doc: CADDocument) -> Optional[PathKey]:
    """Get a root-level part instance key (depth == 1), or None if none exist."""
    root_parts = {k: v for k, v in cad_doc.root_assembly.instances.parts.items() if k.depth == 1}
    if not root_parts:
        return None
    return next(iter(root_parts.keys()))


def get_flexible_assembly_keys(cad_doc: CADDocument) -> list[PathKey]:
    """Get all flexible assembly instance keys."""
    return [
        key
        for key in cad_doc.root_assembly.instances.assemblies
        if cad_doc.root_assembly.instances.is_flexible_assembly(key)
    ]
