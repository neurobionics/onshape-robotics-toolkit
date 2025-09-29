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
    total_assemblies = len(cad.rigid_assembly_instances) + len(cad.flexible_assembly_instances)
    LOGGER.info(
        f"Complete parsing: {len(cad.instances)} instances, {len(cad.parts)} parts, "
        f"{total_assemblies} assemblies ({len(cad.rigid_assembly_instances)} rigid, "
        f"{len(cad.flexible_assembly_instances)} flexible), "
        f"{len(cad.flexible_assembly_data)} flexible assembly data, "
        f"{len(cad.rigid_assembly_data)} rigid assembly data, "
        f"{len(cad.mates)} mates"
    )

    # Show the CAD tree structure
    print("\nCAD Assembly Tree:")
    cad.show_tree()

    # Analysis of potentially confusing properties
    print("\n=== CAD Properties Analysis ===")

    print(f"\nrigid_assembly_instances ({len(cad.rigid_assembly_instances)}):")
    for key, assembly in cad.rigid_assembly_instances.items():
        print(f"  {key} -> {assembly.name} (id: {assembly.id})")

    print(f"\nflexible_assembly_instances ({len(cad.flexible_assembly_instances)}):")
    for key, assembly in cad.flexible_assembly_instances.items():
        print(f"  {key} -> {assembly.name} (id: {assembly.id})")

    print(f"\nflexible_assembly_data ({len(cad.flexible_assembly_data)}):")
    for key, subassembly in cad.flexible_assembly_data.items():
        print(f"  {key} -> {subassembly.uid} (elementId: {subassembly.elementId})")

    print(f"\nrigid_assembly_data ({len(cad.rigid_assembly_data)}):")
    for key, rigid_sub in cad.rigid_assembly_data.items():
        print(f"  {key} -> {rigid_sub.uid} (elementId: {rigid_sub.elementId})")

    print("\n=== Key Relationships Analysis ===")
    print("Comparing rigid_assembly_instances vs rigid_assembly_data:")

    # Check if there's any relationship between rigid assembly instances and rigid assembly data
    rigid_assembly_uids = {assembly.uid for assembly in cad.rigid_assembly_instances.values()}
    rigid_assembly_data_uids = {sub.uid for sub in cad.rigid_assembly_data.values()}

    print(f"Rigid assembly instance UIDs: {rigid_assembly_uids}")
    print(f"Rigid assembly data UIDs: {rigid_assembly_data_uids}")
    print(f"UID overlap: {rigid_assembly_uids & rigid_assembly_data_uids}")

    # Check if there's any relationship between flexible assembly instances and flexible assembly data
    flexible_assembly_uids = {assembly.uid for assembly in cad.flexible_assembly_instances.values()}
    flexible_assembly_data_uids = {sub.uid for sub in cad.flexible_assembly_data.values()}

    print(f"\nFlexible assembly instance UIDs: {flexible_assembly_uids}")
    print(f"Flexible assembly data UIDs: {flexible_assembly_data_uids}")
    print(f"UID overlap: {flexible_assembly_uids & flexible_assembly_data_uids}")

    # Show the convenience properties work correctly
    print("\n=== Convenience Properties ===")
    print(f"Total assembly instances: {len(cad.assembly_instances)}")
    print(f"Total assembly data: {len(cad.assembly_data)}")

    # Transform comparison test
    # compare_transform_methods(cad)
