import mujoco
import mujoco.include
import mujoco.viewer
import numpy as np
from lxml import etree as ET
from scipy.spatial.transform import Rotation
from transformations import compute_motor_torques

from onshape_api.connect import Client
from onshape_api.log import LOGGER, LogLevel
from onshape_api.robot import Robot

# from onshape_api.robot import RobotType, get_robot
from onshape_api.utilities import save_gif

HEIGHT = 480
WIDTH = 640

FREQUENCY = 200
PHASE = 3

KP = 1
KI = 0.1
KD = 0

DEFAULT_COMPILER_ATTRIBUTES = {
    "angle": "radian",
    "meshdir": "meshes",
}

DEFAULT_OPTION_ATTRIBUTES = {
    "timestep": "0.001",
    "gravity": "0 0 -9.81",
    "iterations": "50",
    "solver": "PGS",
}


def run_simulation(model, data, duration, framerate):
    n_frames = int(duration * framerate)
    frames = []

    # visualize contact frames and forces, make body transparent
    options = mujoco.MjvOption()
    mujoco.mjv_defaultOption(options)
    options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
    options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
    options.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = True

    # tweak scales of contact visualization elements
    model.vis.scale.contactwidth = 0.01
    model.vis.scale.contactheight = 0.01
    model.vis.scale.forcewidth = 0.02
    model.vis.map.force = 0.1

    # mujoco.mj_resetData(model, data)
    with mujoco.Renderer(model, HEIGHT, WIDTH) as renderer:
        for _i in range(n_frames):
            mujoco.mj_step(model, data)
            control(data)
            renderer.update_scene(data, "track", scene_option=options)
            pixels = renderer.render()
            frames.append(pixels)

    save_gif(frames, framerate=framerate)
    # show_video(frames, framerate=framerate)


def get_theta(data):
    rot = Rotation.from_quat(data.sensor("orientation").data)
    theta = rot.as_euler("xyz", degrees=False)

    return theta[0], theta[1], theta[2]


def control(data, roll_sp=0, pitch_sp=0):
    roll, pitch, yaw = get_theta(data)
    roll = roll - np.pi

    roll_e = roll - roll_sp
    pitch_e = pitch - pitch_sp

    tx_e = 0
    ty_e = 0

    tx = KP * roll_e + tx_e
    ty = KP * pitch_e + ty_e

    t1, t2, t3 = compute_motor_torques(tx, ty, 0)

    data.ctrl[0] = t1
    data.ctrl[2] = t2
    data.ctrl[1] = t3

    print(f"Roll {roll}, Pitch: {pitch}")


def get_mujoco_element():
    mujoco_element = ET.Element("mujoco")
    ET.SubElement(mujoco_element, "compiler", attrib=DEFAULT_COMPILER_ATTRIBUTES)
    ET.SubElement(mujoco_element, "option", attrib=DEFAULT_OPTION_ATTRIBUTES)

    return mujoco_element


if __name__ == "__main__":
    LOGGER.set_file_name("sim.log")
    LOGGER.set_stream_level(LogLevel.INFO)

    # TODO: Add native support for MJCF (XML) exports: #17
    # ballbot = get_robot(
    #     url="https://cad.onshape.com/documents/1f42f849180e6e5c9abfce52/w/0c00b6520fac5fada24b2104/e/c96b40ef586e60c182f41d29",
    #     robot_name="ballbot",
    #     robot_type=RobotType.MJCF,
    # )
    # ballbot.show()

    client = Client()
    ballbot = Robot.from_url(
        name="ballbot",
        url="https://cad.onshape.com/documents/1f42f849180e6e5c9abfce52/w/0c00b6520fac5fada24b2104/e/c96b40ef586e60c182f41d29",
        client=client,
        max_depth=0,
        use_user_defined_root=True,
    )
    ballbot.show_tree()

    mujoco_attributes = {
        "compiler": DEFAULT_COMPILER_ATTRIBUTES,
        "option": DEFAULT_OPTION_ATTRIBUTES,
    }

    custom_element = get_mujoco_element()
    ballbot.add_custom_element(parent_name="root", element=custom_element)

    ballbot.save(file_path="ballbot.urdf")

    model = mujoco.MjModel.from_xml_path(filename="ballbot.urdf")
    data = mujoco.MjData(model)

    mujoco.mj_resetData(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)

            viewer.sync()
