"""Onshape Robotics Toolkit - A Python library for converting Onshape CAD models to robot description formats.

This package provides tools to:
- Connect to the Onshape API and fetch assembly data
- Parse CAD assemblies into kinematic structures
- Generate URDF and MJCF robot descriptions
- Manipulate and export robot models

Logging Configuration:
    By default, loguru logs to stderr at DEBUG level. For production use, configure logging
    explicitly using the provided helper functions:

    Quick Start:
        >>> from onshape_robotics_toolkit.utilities import setup_default_logging
        >>> setup_default_logging()  # Console (INFO) + File (DEBUG)

    Console Only:
        >>> from onshape_robotics_toolkit.utilities import setup_minimal_logging
        >>> setup_minimal_logging()

    Custom Configuration:
        >>> from loguru import logger
        >>> from onshape_robotics_toolkit.utilities import setup_console_logging, setup_file_logging
        >>> logger.remove()  # Clear all handlers
        >>> setup_console_logging(level="DEBUG")
        >>> setup_file_logging("my_robot.log", rotation="50 MB")

    For more details, see the documentation for the utilities module.
"""

from importlib import metadata as importlib_metadata


def get_version() -> str:
    try:
        return importlib_metadata.version(__name__)
    except importlib_metadata.PackageNotFoundError:  # pragma: no cover
        return "unknown"


__version__: str = get_version()

from onshape_robotics_toolkit.config import (  # noqa: F401 E402
    ToolkitConfig,
    activate_config,
    record_session,
    save_active_session,
)
from onshape_robotics_toolkit.connect import *  # noqa: F403 E402
from onshape_robotics_toolkit.graph import *  # noqa: F403 E402
from onshape_robotics_toolkit.mesh import *  # noqa: F403 E402
from onshape_robotics_toolkit.parse import *  # noqa: F403 E402
from onshape_robotics_toolkit.utilities import *  # noqa: F403 E402
