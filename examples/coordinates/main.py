"""Test hypothesis: Can we compute subassembly-relative transforms from global occurrences?

This script:
1. Fetches the root assembly with global occurrence transforms
2. Fetches a specific subassembly with its local occurrence transforms
3. Saves both occurrence transforms to JSON files
4. Computes subassembly-relative transforms from global transforms
5. Validates the computation against the actual subassembly data
"""

import json
import os

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.log import LOGGER, LogLevel
from onshape_robotics_toolkit.models import Assembly
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.utilities import load_model_from_json, save_model_as_json

# Configuration
USE_CACHED = True
LOG_ASSEMBLY = False
MAX_DEPTH = 1

# Root assembly URL
ROOT_URL = (
    "https://cad.onshape.com/documents/1859bf4489c74b8d9d74e797/w/8e52be2776b88bd0b8524f80/e/46679c6ab890a1b7a6d11a88"
)

# Subassembly URL - paste the URL of a subassembly within the root assembly here
# You can get this by right-clicking on a subassembly in Onshape and copying its link
SUBASSEMBLY_URL = (
    "https://cad.onshape.com/documents/1859bf4489c74b8d9d74e797/w/8e52be2776b88bd0b8524f80/e/996c149cb195c9268ab062b4"
)
SUBASSEMBLY_NAME = "Assembly_2_1"

PARTS_TO_COMPARE = [
    # these parts below are direct children of the subassembly
    "Part_6_1",
    "Part_12_1",
    # these parts below are within a subassembly within this subassembly
    # "Part_10_1",
    # "Part_4_1",
]


def save_occurrence_transforms(cad: CAD, filename: str):
    """Save occurrence transforms and positions to JSON for analysis."""
    data = {}
    for key, occurrence in cad.occurrences.items():
        instance = cad.instances.get(key)
        if instance:
            # Extract position (translation) from transform matrix
            tf_matrix = occurrence.tf
            position = tf_matrix[:3, 3].tolist()  # [x, y, z]

            data[str(key)] = {
                "name": instance.name,
                "transform": occurrence.transform,
                "position": position,
                "name_path": list(key.name_path),
            }

    with open(filename, "w") as f:
        json.dump(data, f, indent=2)
    print(f"Saved occurrence transforms to {filename}")
    return data


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
    subassembly_doc = Document.from_url(url=SUBASSEMBLY_URL)
    client.set_base_url(root_doc.base_url)

    rootassembly = fetch_assembly_json(client, root_doc, "rootassembly.json")
    subassembly = fetch_assembly_json(client, subassembly_doc, "subassembly.json")

    root_cad = CAD.from_assembly(rootassembly, max_depth=MAX_DEPTH)
    sub_cad = CAD.from_assembly(subassembly, max_depth=MAX_DEPTH)

    root_tf_data = save_occurrence_transforms(root_cad, "global_occurrences.json")
    sub_tf_data = save_occurrence_transforms(sub_cad, "local_occurrences.json")

    for part_name in PARTS_TO_COMPARE:
        root_key = root_cad.get_path_key_by_name((SUBASSEMBLY_NAME, part_name))
        root_sub_key = root_cad.get_path_key_by_name(SUBASSEMBLY_NAME)
        sub_key = sub_cad.get_path_key_by_name(part_name)

        root_sub_tf = root_cad.get_transform(root_sub_key)

        root_occ_tf = root_cad.get_transform(root_key, wrt=root_sub_tf)
        sub_occ_tf = sub_cad.get_transform(sub_key)

        print(root_occ_tf, sub_occ_tf)
