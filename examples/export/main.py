from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.graph import create_graph
from onshape_robotics_toolkit.log import LOGGER, LogLevel
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import (
    get_instances,
    get_mates_and_relations,
    get_parts,
    get_subassemblies,
)
from onshape_robotics_toolkit.robot import get_robot
from onshape_robotics_toolkit.utilities.helpers import save_model_as_json

if __name__ == "__main__":
    LOGGER.set_file_name("quadruped.log")
    LOGGER.set_stream_level(LogLevel.INFO)
    client = Client(env=".env")

    document = Document.from_url(
        url="https://cad.onshape.com/documents/cf6b852d2c88d661ac2e17e8/w/c842455c29cc878dc48bdc68/e/b5e293d409dd0b88596181ef"
    )
    client.set_base_url(document.base_url)

    assembly = client.get_assembly(
        did=document.did,
        wtype=document.wtype,
        wid=document.wid,
        eid=document.eid,
        log_response=False,
        with_meta_data=True,
    )

    instances, occurrences, id_to_name_map = get_instances(assembly=assembly, max_depth=0)
    subassemblies, rigid_subassemblies = get_subassemblies(assembly=assembly, client=client, instances=instances)

    parts = get_parts(
        assembly=assembly,
        rigid_subassemblies=rigid_subassemblies,
        client=client,
        instances=instances,
    )
    mates, relations = get_mates_and_relations(
        assembly=assembly,
        subassemblies=subassemblies,
        rigid_subassemblies=rigid_subassemblies,
        id_to_name_map=id_to_name_map,
        parts=parts,
    )

    graph, root_node = create_graph(
        occurrences=occurrences,
        instances=instances,
        parts=parts,
        mates=mates,
        use_user_defined_root=False,
    )

    robot = get_robot(
        assembly=assembly,
        graph=graph,
        root_node=root_node,
        parts=parts,
        mates=mates,
        relations=relations,
        client=client,
        robot_name="quadruped",
    )

    robot.parts = parts
    robot.mates = mates
    robot.relations = relations

    robot.subassemblies = subassemblies
    robot.rigid_subassemblies = rigid_subassemblies

    robot.assembly = assembly

    save_model_as_json(robot.assembly, "quadruped.json")

    robot.show_graph(file_name="quadruped.png")
    robot.save()
