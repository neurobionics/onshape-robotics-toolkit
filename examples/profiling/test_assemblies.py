#!/usr/bin/env python3
"""
Test Assembly Definitions for Performance Profiling

This module defines various test assemblies of different complexities
to validate performance characteristics across different scales.
"""

from dataclasses import dataclass


@dataclass
class TestAssembly:
    """Definition of a test assembly for profiling"""

    name: str
    url: str
    description: str
    timeout_seconds: int = 300  # 5 minute default timeout
    expected_complexity: str = "unknown"  # Optional complexity hint


# Test assembly configurations
# NOTE: Replace these URLs with actual assemblies you have access to
TEST_ASSEMBLIES: list[TestAssembly] = [
    TestAssembly(
        name="quadruped_current",
        url="https://cad.onshape.com/documents/cf6b852d2c88d661ac2e17e8/w/c842455c29cc878dc48bdc68/e/b5e293d409dd0b88596181ef",
        description="Current quadruped robot from main.py",
        expected_complexity="medium",
        timeout_seconds=300,
    ),
    # Add more test assemblies here as you identify them
    # TestAssembly(
    #     name="simple_2dof_arm",
    #     url="https://cad.onshape.com/documents/.../simple_arm",
    #     description="2-DOF robotic arm for baseline testing",
    #     expected_complexity="simple"
    # ),
    # TestAssembly(
    #     name="6dof_manipulator",
    #     url="https://cad.onshape.com/documents/.../6dof_arm",
    #     description="6-DOF industrial manipulator",
    #     expected_complexity="medium"
    # ),
    # TestAssembly(
    #     name="humanoid_robot",
    #     url="https://cad.onshape.com/documents/.../humanoid",
    #     description="Full humanoid robot with sensors",
    #     expected_complexity="complex"
    # ),
    # TestAssembly(
    #     name="large_assembly",
    #     url="https://cad.onshape.com/documents/.../large",
    #     description="Very large assembly for stress testing",
    #     expected_complexity="large",
    #     timeout_seconds=600  # 10 minutes for large assemblies
    # ),
]


def get_assemblies_by_complexity(complexity: str) -> list[TestAssembly]:
    """Get test assemblies filtered by expected complexity level"""
    return [assembly for assembly in TEST_ASSEMBLIES if assembly.expected_complexity == complexity]


def classify_complexity_by_parts(part_count: int) -> str:
    """Automatically classify assembly complexity based on part count"""
    if part_count <= 10:
        return "simple"
    elif part_count <= 50:
        return "medium"
    elif part_count <= 200:
        return "complex"
    else:
        return "large"


def classify_complexity_by_score(complexity_score: float) -> str:
    """Automatically classify assembly complexity based on composite complexity score"""
    if complexity_score <= 50:
        return "simple"
    elif complexity_score <= 200:
        return "medium"
    elif complexity_score <= 800:
        return "complex"
    else:
        return "large"


def get_baseline_assembly() -> TestAssembly:
    """Get the assembly used in the current main.py for baseline testing"""
    return TEST_ASSEMBLIES[0]  # quadruped_current


if __name__ == "__main__":
    print("Test Assembly Configuration")
    print("=" * 50)
    print(f"Total assemblies configured: {len(TEST_ASSEMBLIES)}")
    for assembly in TEST_ASSEMBLIES:
        print(f"  • {assembly.name}: {assembly.expected_complexity} complexity expected")
        print(f"    URL: {assembly.url}")
        print(f"    Description: {assembly.description}")
        print()
    print("Assembly complexity will be automatically determined after processing:")
    print("  • Parts and mates are counted from the robot graph")
    print("  • Nodes = Parts/Links, Edges = Mates/Joints")
    print("  • Complexity is classified based on actual part counts")
    print("\nAdd more test assemblies to TEST_ASSEMBLIES list as needed.")
