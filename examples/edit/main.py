from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.robot import Robot
from onshape_robotics_toolkit.utilities import setup_default_logging

MAX_DEPTH = 2

if __name__ == "__main__":
    setup_default_logging(file_path="edit.log", console_level="INFO")

    client = Client(env=".env")
    document = Document.from_url(
        url="https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/a86aaf34d2f4353288df8812"
    )

    elements = client.get_elements(document.did, document.wtype, document.wid)
    variables = client.get_variables(document.did, document.wid, elements["variables"].id)

    # Update variable expressions
    variables["wheelDiameter"].expression = "180 mm"
    variables["wheelThickness"].expression = "71 mm"
    variables["forkAngle"].expression = "20 deg"

    # Create dictionary with variable names and their new expressions
    variables_to_set = {
        "wheelDiameter": variables["wheelDiameter"].expression,
        "wheelThickness": variables["wheelThickness"].expression,
        "forkAngle": variables["forkAngle"].expression,
    }

    client.set_variables(document.did, document.wid, elements["variables"].id, variables_to_set)
    assembly = client.get_assembly(document.did, document.wtype, document.wid, elements["assembly"].id)

    cad = CAD.from_assembly(assembly, max_depth=MAX_DEPTH, client=client)
    graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
    robot = Robot.from_graph(kinematic_graph=graph, client=client, name=f"edit_{MAX_DEPTH}")
    robot.save()
