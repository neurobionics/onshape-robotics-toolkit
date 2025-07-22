#!/usr/bin/env python3
"""
Clean Performance Benchmarking Script

Simple, focused profiling for CAD -> URDF/MJCF workflow.
No complex analysis, just accurate timing and API call data.
"""

import json
import sys
from datetime import datetime
from pathlib import Path

# Add the toolkit to path
sys.path.append(str(Path(__file__).parent.parent.parent / "src"))

from test_assemblies import TEST_ASSEMBLIES
from workflow_profiler import WorkflowProfiler


def benchmark_assemblies():
    """Benchmark all configured test assemblies"""
    print("Onshape Robotics Toolkit - Performance Benchmark")
    print("=" * 60)
    print("Measuring phase timing and API calls for CAD -> URDF/MJCF workflow.")
    print()

    if not TEST_ASSEMBLIES:
        print("[FAILED] No test assemblies configured!")
        print("Please edit test_assemblies.py to add your Onshape assembly URLs")
        return

    profiler = WorkflowProfiler()

    print(f"Assemblies to benchmark: {len(TEST_ASSEMBLIES)}")
    for assembly in TEST_ASSEMBLIES:
        print(f"  • {assembly.name} ({assembly.complexity}, ~{assembly.expected_parts} parts)")
    print()

    successful = 0
    failed_assemblies = []

    # Benchmark each assembly
    for i, assembly in enumerate(TEST_ASSEMBLIES, 1):
        print(f"[{i}/{len(TEST_ASSEMBLIES)}] Benchmarking: {assembly.name}")
        print(f"  URL: {assembly.url}")

        try:
            profile = profiler.profile_assembly_workflow(assembly.url, assembly.name)

            if profile.success:
                successful += 1
                print(f"  [SUCCESS] Completed in {profile.total_duration:.2f}s")
                print(f"    Parts: {profile.complexity.part_count}")
                print(f"    API calls: {profile.api_call_summary['total_calls']}")
                success_calls = profile.api_call_summary["successful_calls"]
                total_calls = profile.api_call_summary["total_calls"]
                print(f"    Success rate: {success_calls}/{total_calls}")
            else:
                failed_assemblies.append({"name": assembly.name, "errors": profile.error_messages})
                print(f"  [FAILED] Errors: {', '.join(profile.error_messages[:2])}")

        except KeyboardInterrupt:
            print("\n[WARNING] Benchmarking interrupted by user")
            break
        except Exception as e:
            failed_assemblies.append({"name": assembly.name, "errors": [str(e)]})
            print(f"  [FAILED] Exception: {e}")

        print()

    # Generate summary
    print("=" * 60)
    print("BENCHMARK RESULTS")
    print("=" * 60)

    total = len(TEST_ASSEMBLIES)
    failed = len(failed_assemblies)
    success_rate = (successful / total * 100) if total > 0 else 0

    print(f"Total assemblies: {total}")
    print(f"Successful: {successful}")
    print(f"Failed: {failed}")
    print(f"Success rate: {success_rate:.1f}%")

    if successful > 0:
        # Get timing summary
        summary = profiler.get_timing_summary()
        print("\nPhase Performance Summary:")
        print(f"{'Phase':<25} {'Avg Time':<12} {'Avg API Calls':<15} {'Success Rate'}")
        print("-" * 65)

        phase_stats = summary.get("phase_stats", {})
        for phase_name, stats in sorted(phase_stats.items(), key=lambda x: x[1]["avg_time"], reverse=True):
            avg_time = stats["avg_time"]
            avg_api = stats["avg_api_calls"]
            success_rate = stats["success_rate"]
            print(f"{phase_name:<25} {avg_time:<12.3f} {avg_api:<15.1f} {success_rate:<10.1%}")

        # Save benchmark results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        results_file = Path("profiling_results") / f"benchmark_{timestamp}.json"
        results_file.parent.mkdir(exist_ok=True)

        benchmark_data = {
            "benchmark_timestamp": datetime.now().isoformat(),
            "summary": {
                "total_assemblies": total,
                "successful": successful,
                "failed": failed,
                "success_rate": success_rate,
            },
            "phase_stats": phase_stats,
            "failed_assemblies": failed_assemblies,
            "detailed_profiles": [],
        }

        # Add detailed profile data
        for profile in profiler.results:
            profile_data = {
                "assembly_name": profile.assembly_name,
                "assembly_url": profile.assembly_url,
                "success": profile.success,
                "total_duration": profile.total_duration,
                "part_count": profile.complexity.part_count,
                "api_summary": profile.api_call_summary,
                "phases": [],
            }

            for phase in profile.phases:
                profile_data["phases"].append({
                    "phase_name": phase.phase_name,
                    "duration": phase.duration,
                    "success": phase.success,
                    "api_calls": phase.api_calls,
                    "timestamp": phase.timestamp,
                })

            benchmark_data["detailed_profiles"].append(profile_data)

        with results_file.open("w") as f:
            json.dump(benchmark_data, f, indent=2, default=str)

        print(f"\n[SAVED] Benchmark results saved to: {results_file}")

        # Show next steps
        print("\nNext steps:")
        print("1. Review phase timings to identify slowest operations")
        print("2. Generate plots: python plot_results.py --results-dir profiling_results")
        print("3. Use this data as baseline for optimization comparisons")

        # Show top bottlenecks
        sorted_phases = sorted(phase_stats.items(), key=lambda x: x[1]["avg_time"], reverse=True)
        top_3 = sorted_phases[:3]
        if top_3:
            print("\nTop 3 slowest phases:")
            for i, (phase_name, stats) in enumerate(top_3, 1):
                print(f"  {i}. {phase_name}: {stats['avg_time']:.3f}s avg")

    if failed_assemblies:
        print("\nFailed assemblies:")
        for failure in failed_assemblies:
            print(f"  • {failure['name']}: {', '.join(failure['errors'][:2])}")


def main():
    """Main execution"""
    try:
        benchmark_assemblies()
    except KeyboardInterrupt:
        print("\nBenchmarking interrupted by user")
    except Exception as e:
        print(f"Benchmarking failed: {e}")
        raise


if __name__ == "__main__":
    main()
