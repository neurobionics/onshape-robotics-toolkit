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
        name="toy_quadruped",
        url="https://cad.onshape.com/documents/cf6b852d2c88d661ac2e17e8/w/c842455c29cc878dc48bdc68/e/b5e293d409dd0b88596181ef",
        description="Toy Quadruped Robot",
        expected_complexity="medium",
        timeout_seconds=300,
    ),
    TestAssembly(
        name="ballbot",
        url="https://cad.onshape.com/documents/01d73bbd0f243938a11fbb7c/w/20c6ecfe7711055ba2420fdc/e/833959fcd6ba649195a1e94c",
        description="Ball-balancing Robot",
        expected_complexity="simple",
    ),
    TestAssembly(
        name="industrial_quadruped",
        url="https://cad.onshape.com/documents/cb50eea81bddd3a027f4a8b5/w/d547a9baba059204ce4cab33/e/ab953b9b360f5da522daca82",
        description="Industrial Quadruped Robot",
        expected_complexity="complex",
    ),
    TestAssembly(
        name="toy_bike",
        url="https://cad.onshape.com/documents/a1c1addf75444f54b504f25c/w/0d17b8ebb2a4c76be9fff3c7/e/d8f8f1d9dbf9634a39aa7f5b",
        description="Toy Bike",
        expected_complexity="simple",
    ),
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
