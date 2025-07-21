#!/usr/bin/env python3
"""
Advanced example demonstrating direct usage of Rust parsing functions.

This script shows how to use the low-level Rust parsing API directly for
custom workflows and advanced control over the assembly parsing process.
"""

from pathlib import Path

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.graph import create_graph
from onshape_robotics_toolkit.log import LOGGER, LogLevel
from onshape_robotics_toolkit.models.document import Document

# Import Rust components directly
from onshape_robotics_toolkit.native import (
    OnshapeClient as RustClient,
)
from onshape_robotics_toolkit.native import (
    ParseConfig,
    get_performance_metrics,
)
from onshape_robotics_toolkit.parse import (
    get_instances,
    get_mates_and_relations,
    get_parts,
    get_subassemblies,
)
from onshape_robotics_toolkit.robot import Robot, RobotType


def demonstrate_step_by_step_parsing():
    """Demonstrate step-by-step parsing using Rust functions directly."""

    print("🔧 Advanced Rust Parsing Workflow")
    print("=" * 40)

    # Replace with your Onshape document URL
    onshape_url = "https://cad.onshape.com/documents/cf6b852d2c88d661ac2e17e8/w/c842455c29cc878dc48bdc68/e/b5e293d409dd0b88596181ef"

    # Step 1: Setup clients
    print("\n1️⃣ Setting up clients...")
    RustClient(env_file_path=".env")
    python_client = Client(env=".env")

    # Step 2: Parse document and get assembly
    print("2️⃣ Fetching assembly data...")
    doc = Document.from_url(url=onshape_url)

    assembly = python_client.get_assembly(
        did=doc.did,
        wtype=doc.wtype,
        wid=doc.wid,
        eid=doc.eid,
        log_response=False,
        with_meta_data=True,
    )

    print(f"📄 Assembly loaded: {len(assembly.root_assembly.instances)} root instances")

    # Step 3: Parse instances using Rust backend
    print("3️⃣ Parsing instances with Rust backend...")
    max_depth = 1

    instances, occurrences, id_to_name_map = get_instances(assembly=assembly, max_depth=max_depth)

    print(f"📦 Parsed {len(instances)} instances")
    print(f"🎯 Found {len(occurrences)} occurrences")
    print(f"🏷️  Mapped {len(id_to_name_map)} IDs to names")

    # Step 4: Get subassemblies (note: may return empty due to current implementation)
    print("4️⃣ Fetching subassemblies...")
    try:
        subassemblies, rigid_subassemblies = get_subassemblies(
            assembly=assembly, client=python_client, instances=instances
        )
        print(f"🔗 Found {len(subassemblies)} subassemblies")
        print(f"🔒 Found {len(rigid_subassemblies)} rigid subassemblies")
    except Exception as e:
        print(f"⚠️  Subassemblies: {e}")
        subassemblies, rigid_subassemblies = {}, {}

    # Step 5: Get parts (note: may return empty due to current implementation)
    print("5️⃣ Fetching parts...")
    try:
        parts = get_parts(
            assembly=assembly, rigid_subassemblies=rigid_subassemblies, client=python_client, instances=instances
        )
        print(f"⚙️  Found {len(parts)} parts")
    except Exception as e:
        print(f"⚠️  Parts: {e}")
        parts = {}

    # Step 6: Get mates and relations using Rust backend
    print("6️⃣ Parsing mates and relations with Rust backend...")
    try:
        mates, relations = get_mates_and_relations(
            assembly=assembly,
            subassemblies=subassemblies,
            rigid_subassemblies=rigid_subassemblies,
            id_to_name_map=id_to_name_map,
            parts=parts,
        )
        print(f"🔗 Found {len(mates)} mates")
        print(f"📊 Found {len(relations)} relations")
    except Exception as e:
        print(f"⚠️  Mates and relations: {e}")
        mates, relations = {}, {}

    # Step 7: Create graph and robot
    print("7️⃣ Creating robot graph...")
    try:
        graph, root_node = create_graph(
            occurrences=occurrences,
            instances=instances,
            parts=parts,
            mates=mates,
            use_user_defined_root=False,
        )

        print(f"🌐 Graph created with {len(graph.nodes)} nodes and {len(graph.edges)} edges")
        print(f"🌱 Root node: {root_node}")

        # Create robot instance
        robot = Robot(name="advanced_robot", robot_type=RobotType.URDF)
        robot.graph = graph
        robot.parts = parts
        robot.mates = mates
        robot.relations = relations
        robot.assembly = assembly

        # Show structure
        print("\n🔗 Robot structure:")
        robot.show_tree()

    except Exception as e:
        print(f"❌ Graph creation failed: {e}")
        return None
    else:
        return robot


def demonstrate_performance_monitoring():
    """Demonstrate performance monitoring capabilities."""

    print("\n📊 Performance Monitoring")
    print("=" * 30)

    try:
        metrics = get_performance_metrics()
        print("Current performance metrics:")
        for key, value in metrics.items():
            print(f"  - {key}: {value}")
    except Exception as e:
        print(f"⚠️  Performance metrics not available: {e}")


def demonstrate_parse_config():
    """Demonstrate different parsing configurations."""

    print("\n⚙️  Parse Configuration Options")
    print("=" * 35)

    # Show different configuration options
    configs = [
        ParseConfig(max_depth=0, with_mass_properties=False, include_mate_features=True, log_response=False),
        ParseConfig(
            max_depth=2,
            with_mass_properties=True,
            include_mate_features=True,
            include_mate_connectors=True,
            log_response=True,
        ),
    ]

    print("Available ParseConfig options:")
    for i, config in enumerate(configs, 1):
        print(f"  Config {i}:")
        print(f"    - max_depth: {config.max_depth}")
        print(f"    - with_mass_properties: {config.with_mass_properties}")
        print(f"    - include_mate_features: {config.include_mate_features}")
        print(f"    - include_mate_connectors: {config.include_mate_connectors}")
        print(f"    - log_response: {config.log_response}")


def export_robot_to_urdf(robot: Robot):
    """Export the robot to URDF format."""

    if robot is None:
        print("❌ No robot to export")
        return

    print("\n💾 Exporting to URDF...")

    # Create output directory
    output_dir = Path("output")
    output_dir.mkdir(exist_ok=True)

    # Export URDF
    urdf_file = output_dir / "advanced_robot.urdf"

    try:
        robot.save(file_path=str(urdf_file), download_assets=True)

        print(f"✅ URDF exported to: {urdf_file}")

        if urdf_file.exists():
            size = urdf_file.stat().st_size
            print(f"📄 File size: {size} bytes")

    except Exception as e:
        print(f"❌ Export failed: {e}")


def main():
    """Main execution function."""

    # Configure logging
    LOGGER.set_stream_level(LogLevel.INFO)

    print("🚀 Advanced Rust Parsing Example")
    print("=" * 40)

    try:
        # Demonstrate step-by-step parsing
        robot = demonstrate_step_by_step_parsing()

        # Show performance monitoring
        demonstrate_performance_monitoring()

        # Show configuration options
        demonstrate_parse_config()

        # Export to URDF
        export_robot_to_urdf(robot)

        print("\n🎉 Advanced parsing completed!")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        LOGGER.error(f"Advanced parsing failed: {e}")
        raise


if __name__ == "__main__":
    main()
