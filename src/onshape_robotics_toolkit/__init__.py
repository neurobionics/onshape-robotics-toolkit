from importlib import metadata as importlib_metadata


def get_version() -> str:
    try:
        return importlib_metadata.version(__name__)
    except importlib_metadata.PackageNotFoundError:  # pragma: no cover
        return "unknown"


__version__: str = get_version()

# Import all modules
from onshape_robotics_toolkit.connect import *  # noqa: F403 E402
from onshape_robotics_toolkit.graph import *  # noqa: F403 E402
from onshape_robotics_toolkit.log import *  # noqa: F403 E402
from onshape_robotics_toolkit.mesh import *  # noqa: F403 E402
from onshape_robotics_toolkit.native import (  # noqa: E402
    Assembly,
    DefaultWorkspace,
    Document,
    DocumentMetaData,
    Element,
    MassProperties,
    OnshapeClient,
    RootAssembly,
    TranslationJob,
    Variable,
)
from onshape_robotics_toolkit.parse import *  # noqa: F403 E402
from onshape_robotics_toolkit.urdf import *  # noqa: F403 E402
from onshape_robotics_toolkit.utilities import *  # noqa: F403 E402

# Make Rust classes available at package level
__all__ = [
    "Assembly",
    "DefaultWorkspace",
    "Document",
    "DocumentMetaData",
    "Element",
    "MassProperties",
    "OnshapeClient",
    "RootAssembly",
    "TranslationJob",
    "Variable",
]
