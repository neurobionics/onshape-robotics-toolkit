#!/usr/bin/env python3
"""
Simple Baseline Performance Analysis

Focus on accurate timing and API call data for CAD -> URDF/MJCF workflow.
No user impact assessment - just the facts.
"""

import json
import sys
from datetime import datetime
from pathlib import Path

# Add the toolkit to path
sys.path.append(str(Path(__file__).parent.parent.parent / "src"))

from test_assemblies import TEST_ASSEMBLIES
from workflow_profiler import WorkflowProfiler


def run_baseline_analysis():
    """Run baseline analysis on all configured test assemblies"""
    print("Onshape Robotics Toolkit - Baseline Performance Analysis")
    print("=" * 70)
    print("Measuring phase timing and API call patterns for CAD -> URDF/MJCF workflow.")
    print()

    if not TEST_ASSEMBLIES:
        print("[FAILED] No test assemblies configured!")
        print("Please edit test_assemblies.py to add your Onshape assembly URLs")
        return

    profiler = WorkflowProfiler()

    print(f"Configured Test Assemblies: {len(TEST_ASSEMBLIES)}")
    print()

    successful_profiles = 0
    failed_profiles = []

    # Profile each assembly
    for i, assembly in enumerate(TEST_ASSEMBLIES, 1):
        print(f"[{i}/{len(TEST_ASSEMBLIES)}] Profiling: {assembly.name}")
        print(f"  Expected parts: {assembly.expected_parts} ({assembly.complexity})")

        try:
            profile = profiler.profile_assembly_workflow(assembly.url, assembly.name)

            if profile.success:
                successful_profiles += 1
                duration = profile.total_duration
                parts = profile.complexity.part_count
                api_calls = profile.api_call_summary["total_calls"]
                print(f"  [SUCCESS] Success: {duration:.2f}s, {parts} parts, {api_calls} API calls")
            else:
                failed_profiles.append({"assembly": assembly.name, "errors": profile.error_messages})
                print(f"  [FAILED] Failed: {', '.join(profile.error_messages[:2])}")

        except KeyboardInterrupt:
            print("\n[WARNING]  Analysis interrupted by user")
            break
        except Exception as e:
            failed_profiles.append({"assembly": assembly.name, "errors": [str(e)]})
            print(f"  [FAILED] Exception: {e}")

        print()

    # Generate summary
    print(f"{'=' * 70}")
    print("BASELINE ANALYSIS COMPLETE")
    print(f"{'=' * 70}")

    summary = profiler.get_timing_summary()

    print(f"Total profiles: {summary['total_profiles']}")
    print(f"Successful: {summary['successful_profiles']}")
    print(f"Failed: {len(failed_profiles)}")
    success_pct = summary["successful_profiles"] / summary["total_profiles"] * 100
    successful = summary["successful_profiles"]
    total = summary["total_profiles"]
    print(f"Success rate: {successful}/{total} ({success_pct:.1f}%)")

    # Phase performance summary
    phase_stats = summary.get("phase_stats", {})
    if phase_stats:
        print("\n[TIMING]  Phase Performance Summary:")
        headers = (
            f"{'Phase':<20} {'Avg Time':<12} {'Total Time':<12} "
            f"{'Avg API':<10} {'Total API':<10} {'Success Rate':<12}"
        )
        print(headers)
        print("-" * 80)

        # Sort phases by average time (descending)
        sorted_phases = sorted(phase_stats.items(), key=lambda x: x[1]["avg_time"], reverse=True)

        for phase_name, stats in sorted_phases:
            print(
                f"{phase_name:<20} "
                f"{stats['avg_time']:<12.3f} "
                f"{stats['total_time']:<12.3f} "
                f"{stats['avg_api_calls']:<10.1f} "
                f"{stats['total_api_calls']:<10} "
                f"{stats['success_rate']:<12.1%}"
            )

    if failed_profiles:
        print("\n[FAILED] Failed Profiles:")
        for failure in failed_profiles:
            print(f"   • {failure['assembly']}: {', '.join(failure['errors'][:2])}")

    # Save detailed results
    results_data = {
        "analysis_timestamp": datetime.now().isoformat(),
        "summary": {
            "total_assemblies": len(TEST_ASSEMBLIES),
            "successful_profiles": summary["successful_profiles"],
            "failed_profiles": len(failed_profiles),
            "success_rate": (
                summary["successful_profiles"] / summary["total_profiles"] * 100 if summary["total_profiles"] else 0
            ),
        },
        "timing_summary": summary,
        "failed_assemblies": failed_profiles,
        "assembly_profiles": [profile.__dict__ for profile in profiler.results],
    }

    # Convert dataclass objects to dicts for JSON serialization
    for profile_dict in results_data["assembly_profiles"]:
        if "complexity" in profile_dict and hasattr(profile_dict["complexity"], "__dict__"):
            profile_dict["complexity"] = profile_dict["complexity"].__dict__
        if "phases" in profile_dict:
            profile_dict["phases"] = [
                phase.__dict__ if hasattr(phase, "__dict__") else phase for phase in profile_dict["phases"]
            ]

    # Save results
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    results_file = Path("profiling_results") / f"baseline_analysis_{timestamp}.json"
    results_file.parent.mkdir(exist_ok=True)

    with results_file.open("w") as f:
        json.dump(results_data, f, indent=2, default=str)

    print(f"\n[SAVED] Results saved to: {results_file}")

    # Generate plots if data exists
    if successful_profiles > 0:
        print("\n[PROGRESS] To generate visualizations, run:")
        print("   python plot_results.py --results-dir profiling_results --output-dir plots")
        print("\nNext steps:")
        print("1. Review the timing data to identify bottlenecks")
        print("2. Focus optimization efforts on phases with highest avg_time")
        print("3. Monitor API call patterns for efficiency opportunities")
        print("4. Use this as baseline for measuring optimization improvements")


def main():
    """Main execution"""
    try:
        run_baseline_analysis()
    except KeyboardInterrupt:
        print("\nAnalysis interrupted by user")
    except Exception as e:
        print(f"Analysis failed: {e}")
        raise


if __name__ == "__main__":
    main()
