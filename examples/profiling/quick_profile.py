#!/usr/bin/env python3
"""
Quick Profiling Script for Single Assembly

Use this for quick tests before running the full baseline analysis.
"""

import sys
from pathlib import Path

# Add the toolkit to path
sys.path.append(str(Path(__file__).parent.parent.parent / "src"))

from test_assemblies import TEST_ASSEMBLIES
from workflow_profiler import WorkflowProfiler


def _select_assembly(assembly_name: str | None):
    """Select assembly for profiling"""
    if not TEST_ASSEMBLIES:
        print("[FAILED] No test assemblies configured!")
        print("Please edit test_assemblies.py to add your Onshape assembly URLs")
        return None

    if assembly_name:
        for assembly in TEST_ASSEMBLIES:
            if assembly.name == assembly_name:
                return assembly
        print(f"[FAILED] Assembly '{assembly_name}' not found")
        print(f"Available assemblies: {[a.name for a in TEST_ASSEMBLIES]}")
        return None
    return TEST_ASSEMBLIES[0]


def _print_profile_results(profile, analysis):
    """Print profiling results"""
    if not profile.success:
        print("\nProfiling failed")
        print(f"   Errors: {', '.join(profile.error_messages)}")
        return

    print("\nProfiling completed successfully!")
    print(f"   Total time: {profile.total_duration:.2f}s")
    print(f"   Actual parts: {profile.complexity.part_count}")
    print(f"   Phases completed: {len([p for p in profile.phases if p.success])}/{len(profile.phases)}")

    if analysis["bottlenecks"]:
        print("\nIdentified bottlenecks:")
        for phase, bottleneck in analysis["bottlenecks"].items():
            print(f"   - {phase}: {bottleneck['avg_time']:.2f}s ({bottleneck['severity']} priority)")
    else:
        print("\nNo significant bottlenecks identified")


def _print_optimization_suggestions(analysis):
    """Print optimization suggestions based on analysis"""
    print("\nQuick analysis suggests:")
    if analysis["bottlenecks"]:
        high_priority = [p for p, b in analysis["bottlenecks"].items() if b["severity"] == "high"]
        if high_priority:
            print(f"   Rust optimization may be justified for: {', '.join(high_priority)}")
        else:
            print("   Consider simpler optimizations first")
    else:
        print("   Current performance seems good!")


def quick_profile_single_assembly(assembly_name: str | None = None):
    """Profile a single assembly for quick testing"""
    selected_assembly = _select_assembly(assembly_name)
    if not selected_assembly:
        return

    print(f"[PROFILING] Quick Profile: {selected_assembly.name}")
    print(f"   URL: {selected_assembly.url}")
    print(f"   Expected complexity: {selected_assembly.complexity} (~{selected_assembly.expected_parts} parts)")
    print()

    profiler = WorkflowProfiler()

    try:
        profile = profiler.profile_assembly_workflow(selected_assembly.url, selected_assembly.name)

        _print_profile_results(profile)

        if profile.success:
            profiler.save_results(f"quick_profile_{selected_assembly.name}.json")

    except KeyboardInterrupt:
        print("\nProfiling interrupted by user")
    except Exception as e:
        print(f"\nProfiling failed with exception: {e}")


def main():
    """Main execution"""
    import argparse

    parser = argparse.ArgumentParser(description="Quick profile a single assembly")
    parser.add_argument("--assembly", "-a", help="Assembly name to profile (default: first configured)")
    parser.add_argument("--list", action="store_true", help="List available assemblies")

    args = parser.parse_args()

    if args.list:
        print("Available assemblies:")
        for i, assembly in enumerate(TEST_ASSEMBLIES, 1):
            print(f"  {i}. {assembly.name} ({assembly.complexity}) - {assembly.description}")
        return

    quick_profile_single_assembly(args.assembly)


if __name__ == "__main__":
    main()
