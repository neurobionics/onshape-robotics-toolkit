from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.log import LOGGER, LogLevel
from onshape_robotics_toolkit.robot import Robot
from onshape_robotics_toolkit.utilities.helpers import save_model_as_json

TRANSFORMS = (
    "https://cad.onshape.com/documents/1291b9c12b545eeed2d7b739/w/97037c8205b8249da5568aaf/e/9ba9186fc14f9c065161678e"
)
PATTERNS = (
    "https://cad.onshape.com/documents/1291b9c12b545eeed2d7b739/w/97037c8205b8249da5568aaf/e/d882e512e798aab4133f091c"
)
OMNI_WHEEL = (
    "https://cad.onshape.com/documents/8cf89c7fb04f06a2a1a9339c/w/469fb5dc035724877067135b/e/e63ee1f9bd0bd7e034ac2595"
)
TEST_NESTED_TRANSFORMS = (
    "https://cad.onshape.com/documents/1291b9c12b545eeed2d7b739/w/97037c8205b8249da5568aaf/e/7e330c74d3d5bd07649be94a"
)

if __name__ == "__main__":
    LOGGER.set_file_name("test_nested_wo_patterns_depth_1_fixed.log")
    LOGGER.set_stream_level(LogLevel.DEBUG)
    client = Client(env=".env")

    robot = Robot.from_url(
        name="test_nested_wo_patterns_depth_1_fixed",
        url=TEST_NESTED_TRANSFORMS,
        client=client,
        max_depth=1,
        use_user_defined_root=True,
        log_assembly=True,
    )

    save_model_as_json(robot.assembly, "test_nested_wo_patterns_depth_1_fixed.json")

    robot.show_graph(file_name="test_nested_wo_patterns_depth_1_fixed.png")
    robot.save()
