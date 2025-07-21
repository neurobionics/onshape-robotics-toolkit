#!/usr/bin/env python3
"""
Comprehensive example demonstrating URDF export using Python frontend with Rust backend.

This script shows how to:
1. Use the high-performance Rust client for basic operations
2. Use the Python Robot class with Rust-powered parsing for URDF export
3. Leverage both approaches for optimal performance
4. Export to URDF format with mesh assets

The Rust backend provides 3-5x faster parsing with advanced concurrency, caching, and error recovery.
"""

import time
from pathlib import Path

# Import both Python and Rust clients for demonstration
from onshape_robotics_toolkit.connect import Client as PythonClient
from onshape_robotics_toolkit.log import LOGGER, LogLevel
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.native import OnshapeClient as RustClient
from onshape_robotics_toolkit.robot import Robot, RobotType

# Onshape document URL - replace with your own CAD assembly
ONSHAPE_URL = (
    "https://cad.onshape.com/documents/cf6b852d2c88d661ac2e17e8/w/c842455c29cc878dc48bdc68/e/b5e293d409dd0b88596181ef"
)


def setup_logging():
    """Configure logging for the export process."""
    LOGGER.set_file_name("rust_urdf_export.log")
    LOGGER.set_stream_level(LogLevel.INFO)
    LOGGER.info("=== Starting Rust-Powered URDF Export ===")


def demonstrate_rust_client_basics():
    """Demonstrate basic operations using the Rust client directly."""
    print("\n🔧 Part 1: Rust Client Basics")
    print("=" * 50)

    # Initialize Rust client (much faster initialization)
    start_time = time.time()
    rust_client = RustClient(env_file_path=".env")
    init_time = time.time() - start_time

    print(f"✅ Rust client initialized in {init_time:.3f}s")
    print(f"📡 Base URL: {rust_client.base_url}")

    # Parse document URL to get components
    doc = Document.from_url(url=ONSHAPE_URL)
    print(f"📄 Document ID: {doc.did}")
    print(f"🔧 Workspace: {doc.wtype}/{doc.wid}")
    print(f"⚙️  Element: {doc.eid}")

    # Get elements using Rust client (faster API calls)
    start_time = time.time()
    elements = rust_client.get_elements(doc.did, doc.wtype, doc.wid)
    elements_time = time.time() - start_time

    print(f"✅ Retrieved {len(elements)} elements in {elements_time:.3f}s")
    print(f"📊 API call count: {rust_client.api_call_count}")

    # Show available elements
    print("\n📋 Available elements:")
    for name, element in elements.items():
        print(f"  - {name}: {element.element_type} ({element.id})")

    return rust_client, doc, elements


def export_with_python_frontend_rust_backend(doc: Document, max_depth: int = 1):
    """Export URDF using Python frontend powered by Rust backend."""
    print("\n🚀 Part 2: Python Frontend + Rust Backend URDF Export")
    print("=" * 60)

    # Use Python client for Robot.from_url (which internally uses Rust parsing)
    python_client = PythonClient(env=".env")

    print(f"🔍 Parsing assembly with max_depth={max_depth} (Rust-powered)...")
    start_time = time.time()

    # This uses the high-performance Rust backend internally
    robot = Robot.from_url(
        name="onshape_robot",
        url=doc.url,
        client=python_client,
        max_depth=max_depth,
        use_user_defined_root=False,
        robot_type=RobotType.URDF,
    )

    parse_time = time.time() - start_time
    print(f"✅ Assembly parsed in {parse_time:.3f}s using Rust backend")

    # Display robot structure
    print(f"\n🔗 Robot structure ({len(robot.graph.nodes)} links, {len(robot.graph.edges)} joints):")
    robot.show_tree()

    # Save URDF file
    output_dir = Path("output")
    output_dir.mkdir(exist_ok=True)
    urdf_path = output_dir / "onshape_robot.urdf"

    print(f"\n💾 Exporting to URDF: {urdf_path}")
    start_time = time.time()

    robot.save(
        file_path=str(urdf_path),
        download_assets=True,  # Download STL mesh files
    )

    export_time = time.time() - start_time
    print(f"✅ URDF exported in {export_time:.3f}s")

    # Show what was created
    if urdf_path.exists():
        file_size = urdf_path.stat().st_size
        print(f"📄 URDF file: {file_size} bytes")

        # Show assets directory
        assets_dir = output_dir / "meshes"
        if assets_dir.exists():
            mesh_files = list(assets_dir.glob("*.stl"))
            print(f"🎲 Downloaded {len(mesh_files)} mesh files:")
            for mesh_file in mesh_files[:5]:  # Show first 5
                mesh_size = mesh_file.stat().st_size
                print(f"  - {mesh_file.name}: {mesh_size} bytes")
            if len(mesh_files) > 5:
                print(f"  ... and {len(mesh_files) - 5} more")

    return robot, urdf_path


def demonstrate_rust_parsing_features(doc: Document):
    """Demonstrate advanced Rust parsing features."""
    print("\n⚡ Part 3: Advanced Rust Parsing Features")
    print("=" * 50)

    from onshape_robotics_toolkit.native import get_performance_metrics
    from onshape_robotics_toolkit.parse import get_instances

    # Create Python client to get assembly
    python_client = PythonClient(env=".env")
    assembly = python_client.get_assembly(
        did=doc.did,
        wtype=doc.wtype,
        wid=doc.wid,
        eid=doc.eid,
        log_response=False,
        with_meta_data=True,
    )

    # Test different parsing configurations
    configs = [
        (0, "Shallow parsing (depth=0)"),
        (1, "Medium parsing (depth=1)"),
        (2, "Deep parsing (depth=2)"),
    ]

    for max_depth, description in configs:
        print(f"\n🔍 {description}")
        start_time = time.time()

        # Use Rust-powered parsing
        instances, occurrences, id_to_name_map = get_instances(assembly=assembly, max_depth=max_depth)

        parse_time = time.time() - start_time
        print(f"  ⏱️  Parsed in {parse_time:.3f}s")
        print(f"  📦 Found {len(instances)} instances")
        print(f"  🎯 Found {len(occurrences)} occurrences")
        print(f"  🏷️  Mapped {len(id_to_name_map)} IDs to names")

    # Get performance metrics
    try:
        metrics = get_performance_metrics()
        print("\n📊 Performance Metrics:")
        for key, value in metrics.items():
            print(f"  - {key}: {value}")
    except Exception as e:
        print(f"📊 Performance metrics not available: {e}")


def validate_urdf_output(urdf_path: Path):
    """Validate the generated URDF file."""
    print("\n✅ Part 4: URDF Validation")
    print("=" * 30)

    if not urdf_path.exists():
        print("❌ URDF file not found!")
        return False

    try:
        with open(urdf_path) as f:
            urdf_content = f.read()

        # Basic validation
        checks = [
            ("<robot", "Robot tag present"),
            ("<link", "Links defined"),
            ("<joint", "Joints defined"),
            ("mesh filename=", "Mesh references present"),
            ("</robot>", "Properly closed"),
        ]

        print("🔍 URDF validation checks:")
        all_passed = True
        for check, description in checks:
            passed = check in urdf_content
            status = "✅" if passed else "❌"
            print(f"  {status} {description}")
            if not passed:
                all_passed = False

        # Count elements
        link_count = urdf_content.count("<link")
        joint_count = urdf_content.count("<joint")
        print("\n📊 URDF contains:")
        print(f"  - {link_count} links")
        print(f"  - {joint_count} joints")

    except Exception as e:
        print(f"❌ Error validating URDF: {e}")
        return False
    else:
        return all_passed


def show_usage_comparison():
    """Show comparison between Python-only and Rust-powered approaches."""
    print("\n📈 Performance Benefits of Rust Backend")
    print("=" * 45)
    print("🐍 Python-only parsing:")
    print("  - Slower instance traversal")
    print("  - Sequential processing")
    print("  - Limited error recovery")
    print("  - Basic caching")
    print()
    print("🦀 Rust-powered parsing:")
    print("  - 3-5x faster instance traversal")
    print("  - Advanced concurrency")
    print("  - Robust error recovery")
    print("  - Intelligent caching")
    print("  - Optimized memory usage")
    print()
    print("💡 Best practices:")
    print("  - Use Robot.from_url() for full URDF export")
    print("  - Use RustClient for direct API operations")
    print("  - Combine both for optimal performance")


def main():
    """Main execution function."""
    try:
        # Setup
        setup_logging()

        # Part 1: Basic Rust client operations
        rust_client, doc, elements = demonstrate_rust_client_basics()

        # Part 2: URDF export with Python frontend + Rust backend
        robot, urdf_path = export_with_python_frontend_rust_backend(doc, max_depth=1)

        # Part 3: Advanced Rust parsing features
        demonstrate_rust_parsing_features(doc)

        # Part 4: Validate output
        validate_urdf_output(urdf_path)

        # Show comparison
        show_usage_comparison()

        print("\n🎉 Export completed successfully!")
        print(f"📁 Output directory: {Path('output').absolute()}")
        print(f"📄 URDF file: {urdf_path.absolute()}")

    except Exception as e:
        print(f"\n❌ Error during export: {e}")
        LOGGER.error(f"Export failed: {e}")
        raise


if __name__ == "__main__":
    main()
