#!/usr/bin/env python3
"""
Run Baseline Performance Analysis

This script runs comprehensive performance analysis to establish baseline metrics
and identify performance bottlenecks.
"""

import json
import sys
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

# Add the toolkit to path
sys.path.append(str(Path(__file__).parent.parent.parent / "src"))

from test_assemblies import TEST_ASSEMBLIES
from workflow_profiler import WorkflowProfiler


def run_comprehensive_analysis():
    """Run baseline analysis on all configured test assemblies"""
    print("Onshape Robotics Toolkit - Baseline Performance Analysis")
    print("=" * 70)
    print("This analysis will identify performance bottlenecks and timing patterns.")
    print()

    profiler = WorkflowProfiler()

    print(f"Configured Test Assemblies: {len(TEST_ASSEMBLIES)}")
    for i, assembly in enumerate(TEST_ASSEMBLIES, 1):
        print(f"  {i}. {assembly.name} ({assembly.complexity}) - ~{assembly.expected_parts} parts")
    print()

    successful_profiles = 0
    failed_profiles = []

    # Profile each assembly
    for i, assembly in enumerate(TEST_ASSEMBLIES, 1):
        print(f"\n{'=' * 70}")
        print(f"PROFILING {i}/{len(TEST_ASSEMBLIES)}: {assembly.name}")
        print(f"{'=' * 70}")

        try:
            profile = profiler.profile_assembly_workflow(assembly.url, assembly.name)

            if profile.success:
                successful_profiles += 1
                print(f"Successfully profiled {assembly.name}")

                # Show basic performance metrics
                print(f"   Total time: {profile.total_duration:.2f}s")
                print(f"   Part count: {profile.complexity.part_count}")
                print(f"   API calls: {profile.api_call_summary['total_calls']}")

            else:
                failed_profiles.append({"assembly": assembly.name, "errors": profile.error_messages})
                print(f"Failed to profile {assembly.name}")
                print(f"   Errors: {', '.join(profile.error_messages)}")

        except KeyboardInterrupt:
            print("\nAnalysis interrupted by user")
            break
        except Exception as e:
            failed_profiles.append({"assembly": assembly.name, "errors": [str(e)]})
            print(f"Exception profiling {assembly.name}: {e}")

    # Generate comprehensive analysis
    print(f"\n{'=' * 70}")
    print("BASELINE PERFORMANCE ANALYSIS COMPLETE")
    print(f"{'=' * 70}")

    analysis_results = generate_baseline_analysis(profiler, successful_profiles, failed_profiles)

    # Save detailed results
    results_file = save_baseline_results(profiler, analysis_results)

    # Print actionable summary
    print_actionable_summary(analysis_results)

    print(f"\nDetailed results saved to: {results_file}")
    print("Use these results to make evidence-based optimization decisions!")

    return analysis_results


def generate_baseline_analysis(profiler, successful_profiles, failed_profiles):
    """Generate simplified analysis of baseline performance"""
    summary = profiler.get_timing_summary()

    # Enhanced analysis with actionable insights
    enhanced_analysis = {
        "summary": {
            "total_assemblies": len(TEST_ASSEMBLIES),
            "successful_profiles": successful_profiles,
            "failed_profiles": len(failed_profiles),
            "success_rate": successful_profiles / len(TEST_ASSEMBLIES) * 100 if TEST_ASSEMBLIES else 0,
        },
        "timing_summary": summary,
        "failed_assemblies": failed_profiles,
    }

    return enhanced_analysis


def save_baseline_results(profiler, analysis):
    """Save comprehensive baseline results"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"baseline_analysis_{timestamp}.json"

    results_data = {
        "metadata": {
            "analysis_type": "baseline_performance",
            "timestamp": datetime.now().isoformat(),
            "toolkit_version": "python-baseline",
            "python_only": True,
        },
        "raw_profiles": [asdict(profiler.results[i]) for i in range(len(profiler.results))],
        "analysis": analysis,
    }

    results_file = profiler.output_dir / filename
    with open(results_file, "w") as f:
        json.dump(results_data, f, indent=2)

    return results_file


def print_actionable_summary(analysis):
    """Print actionable summary of analysis results"""
    print(f"\n{'ACTIONABLE INSIGHTS':^70}")
    print("=" * 70)

    # Success rate
    success_rate = analysis["summary"]["success_rate"]
    if success_rate < 80:
        print(f"LOW SUCCESS RATE: {success_rate:.1f}% - Focus on reliability improvements first")
    else:
        print(f"Good success rate: {success_rate:.1f}%")

    # Timing summary
    timing_summary = analysis.get("timing_summary", {})
    phase_stats = timing_summary.get("phase_stats", {})

    if phase_stats:
        print("\nTop slowest phases:")
        sorted_phases = sorted(phase_stats.items(), key=lambda x: x[1]["avg_time"], reverse=True)
        for i, (phase_name, stats) in enumerate(sorted_phases[:3], 1):
            print(f"   {i}. {phase_name}: {stats['avg_time']:.3f}s avg")

    # Failed assemblies
    failed = analysis.get("failed_assemblies", [])
    if failed:
        print(f"\nFailed assemblies ({len(failed)}):")
        for failure in failed[:3]:
            print(f"   • {failure['assembly']}: {', '.join(failure['errors'][:2])}")


def main():
    """Main execution"""
    try:
        # Check if we have any test assemblies configured
        if not TEST_ASSEMBLIES:
            print("[FAILED] No test assemblies configured!")
            print("Please edit test_assemblies.py to add your Onshape assembly URLs")
            return

        # Run the analysis
        run_comprehensive_analysis()

        # Provide next steps based on results
        print(f"\n{'NEXT STEPS':^70}")
        print("=" * 70)
        print("1. Review the generated plots and performance report")
        print("2. Use this baseline data to identify optimization targets")
        print("3. Compare results after implementing optimizations")
        print("4. Focus on phases with highest time/API call ratios")

    except KeyboardInterrupt:
        print("\nAnalysis interrupted by user")
    except Exception as e:
        print(f"Analysis failed: {e}")
        raise


if __name__ == "__main__":
    main()
