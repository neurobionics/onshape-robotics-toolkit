from lxml import etree as ET
from onshape_robotics_toolkit.models.mjcf import IMU, Gyro
from onshape_robotics_toolkit.robot import Robot, load_element


def modify_ballbot(ballbot: Robot) -> Robot:
    ballbot.add_light(
        name="light_1",
        directional=True,
        diffuse=[0.6, 0.6, 0.6],
        specular=[0.2, 0.2, 0.2],
        pos=[0, 0, 4],
        direction=[0, 0, -1],
        castshadow=False,
    )

    ballbot.add_actuator(
        actuator_name="motor_1",
        joint_name="Revolute_1",
        ctrl_limited=True,
        ctrl_range=(-50, 50),
        add_encoder=True,
        add_force_sensor=True,
    )
    ballbot.add_actuator(
        actuator_name="motor_2",
        joint_name="Revolute_2",
        ctrl_limited=True,
        ctrl_range=(-50, 50),
        add_encoder=True,
        add_force_sensor=True,
    )
    ballbot.add_actuator(
        actuator_name="motor_3",
        joint_name="Revolute_3",
        ctrl_limited=True,
        ctrl_range=(-50, 50),
        add_encoder=True,
        add_force_sensor=True,
    )

    # For adding to specific named elements (like bodies)
    imu_site = ET.Element("site", name="imu", size="0.01", pos="0 0 0")
    ballbot.add_custom_element_by_name("imu", "Part_3_1", imu_site)

    # Add sensor
    ballbot.add_sensor(
        name="imu",
        sensor=IMU(name="imu", objtype="site", objname="imu"),
    )
    ballbot.add_sensor(
        name="gyro_1",
        sensor=Gyro(name="gyro_1", site="imu"),
    )

    contact = ET.Element("contact")
    pair_1 = ET.SubElement(contact, "pair")
    pair_1.set("geom1", "Part_2_3_collision")
    pair_1.set("geom2", "Part_1_1_collision")
    pair_1.set("friction", "1.75 2.5 0.001 0.001 0.001")

    pair_2 = ET.SubElement(contact, "pair")
    pair_2.set("geom1", "Part_2_2_collision")
    pair_2.set("geom2", "Part_1_1_collision")
    pair_2.set("friction", "1.75 2.5 0.001 0.001 0.001")

    pair_3 = ET.SubElement(contact, "pair")
    pair_3.set("geom1", "Part_2_1_collision")
    pair_3.set("geom2", "Part_1_1_collision")
    pair_3.set("friction", "1.75 2.5 0.001 0.001 0.001")

    pair_4 = ET.SubElement(contact, "pair")
    pair_4.set("geom1", "Part_2_3_collision")
    pair_4.set("geom2", "floor")
    pair_4.set("friction", "0.01 0.9 0.001 0.001 0.001")

    pair_5 = ET.SubElement(contact, "pair")
    pair_5.set("geom1", "Part_2_2_collision")
    pair_5.set("geom2", "floor")
    pair_5.set("friction", "0.01 0.9 0.001 0.001 0.001")

    pair_6 = ET.SubElement(contact, "pair")
    pair_6.set("geom1", "Part_2_1_collision")
    pair_6.set("geom2", "floor")
    pair_6.set("friction", "0.01 0.9 0.001 0.001 0.001")

    pair_7 = ET.SubElement(contact, "pair")
    pair_7.set("geom1", "Part_1_1_collision")
    pair_7.set("geom2", "floor")
    pair_7.set("friction", "1 10 3 10 10")

    ballbot.add_custom_element_by_tag(name="contact", parent_tag="mujoco", element=contact)

    ballbot_mesh = ET.Element("mesh", attrib={"name": "Part_1_1", "file": "meshes\\ball.stl"})
    ballbot.add_custom_element_by_tag(name="ballbot", parent_tag="asset", element=ballbot_mesh)
    ball = load_element("ball.xml")
    ballbot.add_custom_element_by_tag(name="ball", parent_tag="worldbody", element=ball)

    # # set friction="1.0 0.01 0.001" for Part-2-1, Part-2-2, Part-2-3
    # ballbot.set_element_attributes(element_name="Part-2-1-collision", attributes={"friction": "0.1 0.05 0.001"})
    # ballbot.set_element_attributes(element_name="Part-2-2-collision", attributes={"friction": "0.1 0.05 0.001"})
    # ballbot.set_element_attributes(element_name="Part-2-3-collision", attributes={"friction": "0.1 0.05 0.001"})

    ballbot.set_element_attributes(element_name="Revolute_1", attributes={"axis": "0 0 1", "damping": "0.05"})
    ballbot.set_element_attributes(element_name="Revolute_2", attributes={"axis": "0 0 1", "damping": "0.05"})
    ballbot.set_element_attributes(element_name="Revolute_3", attributes={"axis": "0 0 1", "damping": "0.05"})

    return ballbot
