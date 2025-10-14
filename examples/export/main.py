import sys

from loguru import logger

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.robot import Robot

MAX_DEPTH = 2

if __name__ == "__main__":
    # Configure loguru for this example
    logger.add("quadruped.log", rotation="10 MB", level="INFO")
    logger.add(sys.stderr, level="INFO")
    client = Client(env=".env")

    doc = Document.from_url(
        url="https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/d8f8f1d9dbf9634a39aa7f5b"
    )
    assembly = client.get_assembly(doc.did, doc.wtype, doc.wid, doc.eid)

    cad = CAD.from_assembly(assembly, max_depth=MAX_DEPTH, client=client)
    graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
    robot = Robot.from_graph(kinematic_graph=graph, client=client, name=f"edit_{MAX_DEPTH}")
    robot.save(file_path="output/robot", mesh_dir="custom_meshes")
