from importlib import metadata as importlib_metadata


def get_version() -> str:
    try:
        return importlib_metadata.version(__name__)
    except importlib_metadata.PackageNotFoundError:  # pragma: no cover
        return "unknown"


__version__: str = get_version()

from onshape_robotics_toolkit.connect import *  # noqa: F403 E402
from onshape_robotics_toolkit.graph import *  # noqa: F403 E402
from onshape_robotics_toolkit.log import *  # noqa: F403 E402
from onshape_robotics_toolkit.mesh import *  # noqa: F403 E402
from onshape_robotics_toolkit.parse import *  # noqa: F403 E402
from onshape_robotics_toolkit.utilities import *  # noqa: F403 E402
