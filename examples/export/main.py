from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.log import LOGGER, LogLevel
from onshape_robotics_toolkit.robot import Robot
from onshape_robotics_toolkit.utilities.helpers import save_model_as_json

if __name__ == "__main__":
    LOGGER.set_file_name("quadruped.log")
    LOGGER.set_stream_level(LogLevel.INFO)
    client = Client(env=".env")

    robot = Robot.from_url(
        name="quadruped",
        url="https://cad.onshape.com/documents/cf6b852d2c88d661ac2e17e8/w/c842455c29cc878dc48bdc68/e/b5e293d409dd0b88596181ef",
        client=client,
        max_depth=0,
        use_user_defined_root=False,
    )

    save_model_as_json(robot.assembly, "quadruped.json")

    robot.show_graph(file_name="quadruped.png")
    robot.save()
