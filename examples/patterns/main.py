from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.log import LOGGER, LogLevel
from onshape_robotics_toolkit.robot import Robot
from onshape_robotics_toolkit.utilities.helpers import save_model_as_json

if __name__ == "__main__":
    LOGGER.set_file_name("patterns.log")
    LOGGER.set_stream_level(LogLevel.CRITICAL)
    client = Client(env=".env")

    robot = Robot.from_url(
        name="patterns",
        url="https://cad.onshape.com/documents/1291b9c12b545eeed2d7b739/w/97037c8205b8249da5568aaf/e/b90f507e0ecc91fb355c64c9",
        client=client,
        max_depth=0,
        use_user_defined_root=False,
    )

    save_model_as_json(robot.assembly, "patterns.json")

    robot.show_graph(file_name="patterns.png")
    robot.save()
