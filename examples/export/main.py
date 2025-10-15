from pathlib import Path

from onshape_robotics_toolkit.config import ORTConfig
from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.formats import MJCFSerializer
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.robot import Robot
from onshape_robotics_toolkit.utilities import setup_default_logging

# Basic parameters a user would normally hard-code or source from their own scripts.
ENV_PATH = ".env"
DOCUMENT_URL = (
    "https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/"
    "w/0d17b8ebb2a4c76be9fff3c7/e/d8f8f1d9dbf9634a39aa7f5b"
)
MAX_DEPTH = 2
EXPORT_PATH = Path("output/robot.xml")
MESH_DIR = "custom_meshes"
CONFIG_OUTPUT = Path("ORT.yaml")


def main() -> None:
    setup_default_logging(console_level="INFO", file_level="DEBUG", file_path="export.log")

    config = ORTConfig.load(CONFIG_OUTPUT) if CONFIG_OUTPUT.exists() else None

    env_path = config.client.env if config and config.client and config.client.env is not None else ENV_PATH
    base_url = config.client.base_url if config and config.client else None
    max_depth = config.cad.max_depth if config and config.cad else MAX_DEPTH
    use_root = config.kinematics.use_user_defined_root if config and config.kinematics else True
    robot_name = config.robot.name if config and config.robot else f"export_{max_depth}"
    export_path = config.export.file_path if config and config.export and config.export.file_path else str(EXPORT_PATH)
    download_assets = config.export.download_assets if config and config.export else True
    mesh_dir = config.export.mesh_dir if config and config.export and config.export.mesh_dir else MESH_DIR

    client = Client(env=env_path, base_url=base_url) if base_url else Client(env=env_path)
    cad = CAD.from_url(DOCUMENT_URL, client=client, max_depth=max_depth)
    graph = KinematicGraph.from_cad(cad, use_user_defined_root=use_root)
    robot = Robot.from_graph(kinematic_graph=graph, client=client, name=robot_name)

    # Export using MJCF serializer - no need to import MJCFConfig!
    serializer = MJCFSerializer()
    serializer.save(
        robot,
        export_path,
        download_assets=download_assets,
        mesh_dir=mesh_dir,
        position=(0, 0, 0),
        add_ground_plane=True,
    )


if __name__ == "__main__":
    main()
