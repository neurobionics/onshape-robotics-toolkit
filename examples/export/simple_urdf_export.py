#!/usr/bin/env python3
"""
Simple URDF export example using Python frontend with Rust backend.

This script demonstrates the most straightforward way to export an Onshape CAD
assembly to URDF format using the high-performance Rust backend.
"""

from pathlib import Path

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.log import LOGGER, LogLevel
from onshape_robotics_toolkit.robot import Robot, RobotType


def main():
    """Export Onshape assembly to URDF using Rust-powered parsing."""

    # Configure logging
    LOGGER.set_stream_level(LogLevel.INFO)

    # Replace with your Onshape document URL
    onshape_url = "https://cad.onshape.com/documents/cf6b852d2c88d661ac2e17e8/w/c842455c29cc878dc48bdc68/e/b5e293d409dd0b88596181ef"

    print("🚀 Starting URDF export with Rust backend...")

    # Initialize client (reads credentials from .env file)
    client = Client(env=".env")

    # Create robot from Onshape URL (uses Rust backend internally)
    print("🔍 Parsing Onshape assembly...")
    robot = Robot.from_url(
        name="my_robot",  # Robot name
        url=onshape_url,  # Onshape document URL
        client=client,  # Client for API calls
        max_depth=1,  # Assembly traversal depth
        use_user_defined_root=False,
        robot_type=RobotType.URDF,
    )

    print(f"✅ Found {len(robot.graph.nodes)} links and {len(robot.graph.edges)} joints")

    # Show robot structure
    print("\n🔗 Robot structure:")
    robot.show_tree()

    # Create output directory
    output_dir = Path("output")
    output_dir.mkdir(exist_ok=True)

    # Export to URDF with mesh files
    urdf_file = output_dir / "my_robot.urdf"
    print(f"\n💾 Exporting to: {urdf_file}")

    robot.save(
        file_path=str(urdf_file),
        download_assets=True,  # Downloads STL mesh files
    )

    print("🎉 URDF export completed!")
    print(f"📁 Files saved to: {output_dir.absolute()}")

    # Show generated files
    if urdf_file.exists():
        print(f"📄 URDF file: {urdf_file.name} ({urdf_file.stat().st_size} bytes)")

    meshes_dir = output_dir / "meshes"
    if meshes_dir.exists():
        mesh_files = list(meshes_dir.glob("*.stl"))
        print(f"🎲 Mesh files: {len(mesh_files)} STL files downloaded")


if __name__ == "__main__":
    main()
