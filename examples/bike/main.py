import onshape_api as osa
from onshape_api.graph import create_graph, get_urdf_components
from onshape_api.models.robot import Robot
from onshape_api.parse import (
    get_instances,
    get_mass_properties,
    get_mates,
    get_occurences,
    get_parts,
    get_subassemblies,
)

# Initialize the client with the constructed path
client = osa.Client()
doc = osa.Document.from_url(
    url="https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/a86aaf34d2f4353288df8812"
)

elements = client.get_elements(doc.did, doc.wtype, doc.wid)
variables = client.get_variables(doc.did, doc.wid, elements["variables"].id)

variables["wheelDiameter"].expression = "300 mm"
variables["wheelThickness"].expression = "71 mm"
variables["forkAngle"].expression = "20 deg"

client.set_variables(doc.did, doc.wid, elements["variables"].id, variables)
assembly, _ = client.get_assembly(doc.did, doc.wtype, doc.wid, elements["assembly"].id)

occurences = get_occurences(assembly)
instances = get_instances(assembly)
subassemblies = get_subassemblies(assembly, instances)
parts = get_parts(assembly, instances)
mass_properties = get_mass_properties(parts, doc.wid, client)
mates = get_mates(assembly)

graph = create_graph(occurences=occurences, instances=instances, parts=parts, mates=mates, directed=False)
# show_graph(graph)

links, joints = get_urdf_components(graph, doc.wid, parts, mass_properties, mates, client)

robot = Robot(name="bike", links=links, joints=joints)
robot.save("bike.urdf")
