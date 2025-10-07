"""Test hypothesis: Can we compute subassembly-relative transforms from global occurrences?

This script:
1. Fetches the root assembly with global occurrence transforms
2. Fetches a specific subassembly with its local occurrence transforms
3. Saves both occurrence transforms to JSON files
4. Computes subassembly-relative transforms from global transforms
5. Validates the computation against the actual subassembly data
"""

import os

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.log import LOGGER, LogLevel
from onshape_robotics_toolkit.models import Assembly
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.robot import Robot
from onshape_robotics_toolkit.utilities import load_model_from_json, save_model_as_json

# Configuration
USE_CACHED = False
LOG_ASSEMBLY = False
MAX_DEPTH = 1

# Root assembly URL
ROOT_URL = (
    "https://cad.onshape.com/documents/9c982cc66e2d3357ecf31371/w/21b699e5966180f4906fb6d1/e/3ef3c81430ecd68387fc3962"
)


def fetch_assembly_json(client: Client, doc: Document, cache_file: str) -> Assembly:
    """Fetch assembly from Onshape or load from cache."""
    if USE_CACHED and os.path.exists(cache_file):
        assembly = load_model_from_json(Assembly, cache_file)
        print(f"Loaded cached assembly from {cache_file}")
    else:
        assembly = client.get_assembly(
            did=doc.did,
            wtype=doc.wtype,
            wid=doc.wid,
            eid=doc.eid,
            log_response=LOG_ASSEMBLY,
            with_meta_data=True,
        )
        save_model_as_json(assembly, cache_file)
        print(f"Fetched and cached assembly to {cache_file}")
    return assembly


if __name__ == "__main__":
    LOGGER.set_file_name("occurrence_test.log")
    LOGGER.set_stream_level(LogLevel.INFO)

    client = Client(env=".env")

    root_doc = Document.from_url(url=ROOT_URL)
    client.set_base_url(root_doc.base_url)

    rootassembly = fetch_assembly_json(client, root_doc, "rootassembly.json")

    cad = CAD.from_assembly(rootassembly, max_depth=MAX_DEPTH)
    graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)

    robot = Robot.from_graph(cad=cad, kinematic_graph=graph, client=client, name="rootassembly")
    robot.save()
