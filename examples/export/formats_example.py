"""
Example demonstrating the new formats API for exporting robots.

This example shows two approaches for using the serializers:
1. Simple approach: Pass options directly as kwargs (no need to import MJCFConfig)
2. Advanced approach: Use MJCFConfig for complex setups with lights, actuators, etc.
"""

from pathlib import Path

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.formats import MJCFConfig, MJCFSerializer, URDFSerializer
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.models.mjcf import Actuator, Light
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.robot import Robot
from onshape_robotics_toolkit.utilities import setup_default_logging

# Configuration
ENV_PATH = ".env"
DOCUMENT_URL = (
    "https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/"
    "w/0d17b8ebb2a4c76be9fff3c7/e/d8f8f1d9dbf9634a39aa7f5b"
)
MAX_DEPTH = 2
OUTPUT_DIR = Path("output/formats_example")
MESH_DIR = "meshes"


def main() -> None:
    """Demonstrate the new formats API."""
    setup_default_logging(console_level="INFO", file_level="DEBUG", file_path="formats_example.log")

    # Step 1: Generate robot from Onshape CAD
    print("\n" + "=" * 60)
    print("Step 1: Generating robot from Onshape CAD...")
    print("=" * 60)

    client = Client(env=ENV_PATH)
    cad = CAD.from_url(DOCUMENT_URL, client=client, max_depth=MAX_DEPTH)
    graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
    robot = Robot.from_graph(kinematic_graph=graph, client=client, name="formats_demo")

    print(f"✓ Robot '{robot.name}' created with {len(robot.nodes)} links and {len(robot.edges)} joints")

    # Step 2: Export to URDF using the new URDFSerializer
    print("\n" + "=" * 60)
    print("Step 2: Exporting to URDF using URDFSerializer...")
    print("=" * 60)

    urdf_serializer = URDFSerializer()
    urdf_path = OUTPUT_DIR / "robot.urdf"
    urdf_serializer.save(robot, str(urdf_path), download_assets=True, mesh_dir=MESH_DIR)

    print(f"✓ URDF saved to: {urdf_path.absolute()}")

    # Step 3: Export to MJCF - Simple approach (no MJCFConfig needed!)
    print("\n" + "=" * 60)
    print("Step 3: Exporting to MJCF - Simple approach (no MJCFConfig)...")
    print("=" * 60)

    mjcf_simple_serializer = MJCFSerializer()
    mjcf_simple_path = OUTPUT_DIR / "robot_simple.xml"
    # Just pass options directly as kwargs!
    mjcf_simple_serializer.save(
        robot,
        str(mjcf_simple_path),
        download_assets=False,  # Reuse meshes from URDF export
        position=(0, 0, 1),  # Robot starts 1m above ground
        add_ground_plane=True,
        ground_plane_size=10,
    )

    print(f"✓ Simple MJCF saved to: {mjcf_simple_path.absolute()}")
    print("  (No MJCFConfig import needed - just pass options as kwargs!)")

    # Step 4: Export to MJCF with advanced customization
    print("\n" + "=" * 60)
    print("Step 4: Exporting to MJCF with advanced customization...")
    print("=" * 60)

    # Create advanced configuration
    mjcf_advanced_config = MJCFConfig(
        position=(0, 0, 0.5),
        ground_position=(0, 0, 0),
        compiler_attributes={
            "angle": "radian",
            "eulerseq": "xyz",
        },
        option_attributes={
            "timestep": "0.002",  # 2ms timestep
            "gravity": "0 0 -9.81",
            "iterations": "100",  # More solver iterations
        },
        add_ground_plane=True,
        ground_plane_size=15,
    )

    # Add a directional light (sun)
    mjcf_advanced_config.lights["sun"] = Light(
        directional=True,
        diffuse=(0.8, 0.8, 0.8),
        specular=(0.3, 0.3, 0.3),
        pos=(0, 0, 10),
        direction=(0, 0, -1),
        castshadow=True,
    )

    # Add actuators to all non-fixed joints
    for parent_key, child_key in robot.edges:
        edge_data = robot.get_edge_data(parent_key, child_key)
        if edge_data:
            joint = edge_data.get("data")
            if joint and hasattr(joint, "joint_type") and joint.joint_type != "fixed":
                actuator_name = f"{joint.name}_motor"
                mjcf_advanced_config.actuators[actuator_name] = Actuator(
                    name=actuator_name,
                    joint=joint.name,
                    ctrllimited=True,
                    ctrlrange=(-1.0, 1.0),
                    gear=50.0,
                )
                print(f"  Added actuator: {actuator_name} for joint: {joint.name}")

    # Serialize with advanced config
    mjcf_advanced_serializer = MJCFSerializer(mjcf_advanced_config)
    mjcf_advanced_path = OUTPUT_DIR / "robot_advanced.xml"
    mjcf_advanced_serializer.save(robot, str(mjcf_advanced_path), download_assets=False)

    print(f"✓ Advanced MJCF saved to: {mjcf_advanced_path.absolute()}")

    # Step 5: Demonstrate serialization to string (for in-memory use)
    print("\n" + "=" * 60)
    print("Step 5: Demonstrating in-memory serialization...")
    print("=" * 60)

    urdf_string = urdf_serializer.serialize(robot)
    mjcf_string = mjcf_simple_serializer.serialize(robot, position=(0, 0, 1), add_ground_plane=True)

    print(f"✓ URDF string length: {len(urdf_string)} characters")
    print(f"✓ MJCF string length: {len(mjcf_string)} characters")
    print("\nFirst 200 characters of URDF:")
    print(urdf_string[:200] + "...")

    # Summary
    print("\n" + "=" * 60)
    print("Summary")
    print("=" * 60)
    print(f"✓ Generated {robot.name} with {len(robot.nodes)} links and {len(robot.edges)} joints")
    print("✓ Exported 3 files:")
    print(f"  1. URDF: {urdf_path.name}")
    print(f"  2. Simple MJCF: {mjcf_simple_path.name} (using kwargs)")
    actuator_count = len(mjcf_advanced_config.actuators)
    print(f"  3. Advanced MJCF: {mjcf_advanced_path.name} (using MJCFConfig with {actuator_count} actuators)")
    print(f"✓ All files saved to: {OUTPUT_DIR.absolute()}")
    print("\n" + "=" * 60)


if __name__ == "__main__":
    main()
