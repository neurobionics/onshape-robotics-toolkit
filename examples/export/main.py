from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.log import LOGGER, LogLevel
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.robot import Robot

MAX_DEPTH = 2

if __name__ == "__main__":
    LOGGER.set_file_name("quadruped.log")
    LOGGER.set_stream_level(LogLevel.INFO)
    client = Client(env=".env")

    doc = Document.from_url(
        url="https://cad.onshape.com/documents/cf6b852d2c88d661ac2e17e8/w/c842455c29cc878dc48bdc68/e/b5e293d409dd0b88596181ef"
    )
    assembly = client.get_assembly(doc.did, doc.wtype, doc.wid, doc.eid)

    cad = CAD.from_assembly(assembly, max_depth=MAX_DEPTH, client=client)
    graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
    robot = Robot.from_graph(kinematic_graph=graph, client=client, name=f"edit_{MAX_DEPTH}")
    robot.save(file_path="output/robot", mesh_dir="custom_meshes")
