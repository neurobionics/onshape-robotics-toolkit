import sys
from importlib import metadata as importlib_metadata

from loguru import logger


def get_version() -> str:
    try:
        return importlib_metadata.version(__name__)
    except importlib_metadata.PackageNotFoundError:  # pragma: no cover
        return "unknown"


__version__: str = get_version()

# Configure loguru with sensible defaults
# By default, loguru logs to stderr. We keep that but also add file logging.
# Remove the default stderr handler
logger.remove()

# Add console handler at INFO level
logger.add(
    sys.stderr,
    format="<green>{time:HH:mm:ss}</green> | <level>{level: <8}</level> | \
        <cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> - <level>{message}</level>",
    level="INFO",
    colorize=True,
)

# Add file handler at DEBUG level
logger.add(
    "onshape_toolkit.log",
    format="{time:HH:mm:ss} | {level: <8} | {name}:{function}:{line} - {message}",
    level="DEBUG",
    rotation="10 MB",
    retention="7 days",
    compression="zip",
    enqueue=True,  # Thread-safe logging
    delay=True,  # Delay file creation until the first log message
)

# Note: loguru comes pre-configured with stderr output at DEBUG level
# Users can customize by calling logger.remove() and logger.add() with their preferred configuration

from onshape_robotics_toolkit.connect import *  # noqa: F403 E402
from onshape_robotics_toolkit.graph import *  # noqa: F403 E402
from onshape_robotics_toolkit.mesh import *  # noqa: F403 E402
from onshape_robotics_toolkit.parse import *  # noqa: F403 E402
from onshape_robotics_toolkit.utilities import *  # noqa: F403 E402
