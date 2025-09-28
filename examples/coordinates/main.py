import os
import pickle

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.log import LOGGER, LogLevel
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import (
    CAD,
)

TRANSFORMS = (
    "https://cad.onshape.com/documents/1859bf4489c74b8d9d74e797/w/8e52be2776b88bd0b8524f80/e/46679c6ab890a1b7a6d11a88"
)

DEPTH = 0
NAME = f"transforms_{DEPTH}"
USE_CACHED = True  # Set to False to force fresh API call
PICKLE_PATH = f"{NAME}.pkl"
LOG_ASSEMBLY = False


def get_pickled_assembly():
    if USE_CACHED and os.path.exists(PICKLE_PATH):
        LOGGER.info("Loading assembly from cached pickle file...")
        with open(PICKLE_PATH, "rb") as f:
            assembly = pickle.load(f)  # noqa: S301
    else:
        LOGGER.info("Fetching assembly from Onshape API...")
        assembly = client.get_assembly(
            did=document.did,
            wtype=document.wtype,
            wid=document.wid,
            eid=document.eid,
            log_response=LOG_ASSEMBLY,
            with_meta_data=True,
        )
        # Cache for next time
        with open(PICKLE_PATH, "wb") as f:
            pickle.dump(assembly, f)
        LOGGER.info(f"Assembly cached to {PICKLE_PATH}")

    return assembly


if __name__ == "__main__":
    LOGGER.set_file_name(f"{NAME}.log")
    LOGGER.set_stream_level(LogLevel.INFO)

    # Initialize client
    client = Client(env=".env")

    document = Document.from_url(url=TRANSFORMS)
    client.set_base_url(document.base_url)
    assembly = get_pickled_assembly()

    cad = CAD.from_assembly(assembly, client, max_depth=DEPTH)
    total_assemblies = len(cad.rigid_assemblies) + len(cad.flexible_assemblies)
    LOGGER.info(
        f"Complete parsing: {len(cad.instances)} instances, {len(cad.parts)} parts, "
        f"{total_assemblies} assemblies ({len(cad.rigid_assemblies)} rigid, "
        f"{len(cad.flexible_assemblies)} flexible), "
        f"{len(cad.subassemblies)} flexible subassemblies, "
        f"{len(cad.rigid_subassemblies)} rigid subassemblies, "
        f"{len(cad.mates)} mates"
    )

    # Show the CAD tree structure
    print("\nCAD Assembly Tree:")
    cad.show_tree()
    # Transform comparison test
    # compare_transform_methods(cad)
