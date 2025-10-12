"""Pytest fixtures for the onshape-robotics-toolkit test suite."""

from __future__ import annotations

import sys
import types
from pathlib import Path

import pytest

from onshape_robotics_toolkit.models.assembly import Assembly
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.utilities import load_model_from_json

# Ensure the project root is on sys.path when pytest uses importlib mode.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:  # pragma: no cover - safety guard
    sys.path.insert(0, str(PROJECT_ROOT))

# The top-level package imports ``stl`` (numpy-stl). We don't need the real dependency
# for the unit tests that run in CI, so we provide a very small stub before importing
# the toolkit. This keeps imports lightweight and avoids optional dependency failures.
if "stl" not in sys.modules:  # pragma: no cover - defensive guard
    stub = types.ModuleType("stl")

    class _Mesh:  # minimal interface used in the codebase
        def __init__(self, *args, **kwargs):
            pass

        @classmethod
        def from_file(cls, *args, **kwargs):
            return cls()

        def save(self, *args, **kwargs):
            return None

    mesh_module = types.ModuleType("stl.mesh")
    mesh_module.Mesh = _Mesh

    stub.mesh = mesh_module
    sys.modules["stl"] = stub
    sys.modules["stl.mesh"] = mesh_module

# Matplotlib shipped with the sandbox is compiled against NumPy 1.x. To avoid the
# compatibility issue when running on NumPy 2, we provide a lightweight stub that
# satisfies the small subset of attributes the toolkit imports during tests.
if "matplotlib" not in sys.modules:  # pragma: no cover - defensive guard
    mpl = types.ModuleType("matplotlib")
    animation_mod = types.ModuleType("matplotlib.animation")
    pyplot_mod = types.ModuleType("matplotlib.pyplot")

    def _noop(*args, **kwargs):
        return None

    # Provide minimal surface area used inside helpers.py
    animation_mod.FuncAnimation = _noop
    pyplot_mod.figure = _noop
    pyplot_mod.savefig = _noop
    pyplot_mod.close = _noop

    mpl.animation = animation_mod
    mpl.pyplot = pyplot_mod

    sys.modules["matplotlib"] = mpl
    sys.modules["matplotlib.animation"] = animation_mod
    sys.modules["matplotlib.pyplot"] = pyplot_mod


@pytest.fixture
def assembly_json_path() -> Path:
    """Path to the test assembly JSON file."""
    return Path(__file__).parent / "data" / "assembly.json"


@pytest.fixture
def assembly(assembly_json_path: Path) -> Assembly:
    """Load assembly from JSON file."""
    return load_model_from_json(Assembly, str(assembly_json_path))


@pytest.fixture
def cad_doc(assembly: Assembly) -> CAD:
    """Create CAD from assembly with max_depth=2 (all flexible for this test assembly)."""
    return CAD.from_assembly(assembly, max_depth=2)


@pytest.fixture
def cad_doc_depth_1(assembly: Assembly) -> CAD:
    """Create CAD from assembly with max_depth=1."""
    return CAD.from_assembly(assembly, max_depth=1)


@pytest.fixture
def cad_doc_depth_0(assembly: Assembly) -> CAD:
    """Create CAD from assembly with max_depth=0 (all nested assemblies rigid)."""
    return CAD.from_assembly(assembly, max_depth=0)
