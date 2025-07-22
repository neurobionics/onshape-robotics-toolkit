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
    expected_parts: int
    complexity: str  # "simple", "medium", "complex", "large"
    description: str
    timeout_seconds: int = 300  # 5 minute default timeout


# Test assembly configurations
# NOTE: Replace these URLs with actual assemblies you have access to
TEST_ASSEMBLIES: list[TestAssembly] = [
    TestAssembly(
        name="quadruped_current",
        url="https://cad.onshape.com/documents/cf6b852d2c88d661ac2e17e8/w/c842455c29cc878dc48bdc68/e/b5e293d409dd0b88596181ef",
        expected_parts=20,
        complexity="medium",
        description="Current quadruped robot from main.py",
        timeout_seconds=300,
    ),
    # Add more test assemblies here as you identify them
    # TestAssembly(
    #     name="simple_2dof_arm",
    #     url="https://cad.onshape.com/documents/.../simple_arm",
    #     expected_parts=5,
    #     complexity="simple",
    #     description="2-DOF robotic arm for baseline testing"
    # ),
    # TestAssembly(
    #     name="6dof_manipulator",
    #     url="https://cad.onshape.com/documents/.../6dof_arm",
    #     expected_parts=25,
    #     complexity="medium",
    #     description="6-DOF industrial manipulator"
    # ),
    # TestAssembly(
    #     name="humanoid_robot",
    #     url="https://cad.onshape.com/documents/.../humanoid",
    #     expected_parts=200,
    #     complexity="complex",
    #     description="Full humanoid robot with sensors"
    # ),
    # TestAssembly(
    #     name="large_assembly",
    #     url="https://cad.onshape.com/documents/.../large",
    #     expected_parts=500,
    #     complexity="large",
    #     description="Very large assembly for stress testing",
    #     timeout_seconds=600  # 10 minutes for large assemblies
    # ),
]


def get_assemblies_by_complexity(complexity: str) -> list[TestAssembly]:
    """Get test assemblies filtered by complexity level"""
    return [assembly for assembly in TEST_ASSEMBLIES if assembly.complexity == complexity]


def get_baseline_assembly() -> TestAssembly:
    """Get the assembly used in the current main.py for baseline testing"""
    return TEST_ASSEMBLIES[0]  # quadruped_current


if __name__ == "__main__":
    print("Test Assembly Configuration")
    print("=" * 50)
    print(f"Total assemblies configured: {len(TEST_ASSEMBLIES)}")
    for assembly in TEST_ASSEMBLIES:
        print(f"  • {assembly.name}: {assembly.expected_parts} parts ({assembly.complexity})")
    print("\nAdd more test assemblies to TEST_ASSEMBLIES list as needed.")
