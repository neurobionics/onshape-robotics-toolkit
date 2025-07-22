#!/usr/bin/env python3
"""
Simplified profiling runner

Focus on accurate phase timing and API call tracking for CAD -> URDF/MJCF workflow.
"""

from test_assemblies import TEST_ASSEMBLIES
from workflow_profiler import WorkflowProfiler


def run_single_assembly_profile(assembly_name: str | None = None):
    """Profile a single assembly with simplified output"""
    if not TEST_ASSEMBLIES:
        print("[FAILED] No test assemblies configured!")
        print("Please edit test_assemblies.py to add your Onshape assembly URLs")
        return False

    # Select assembly
    if assembly_name:
        selected_assembly = None
        for assembly in TEST_ASSEMBLIES:
            if assembly.name == assembly_name:
                selected_assembly = assembly
                break
        if not selected_assembly:
            print(f"[FAILED] Assembly '{assembly_name}' not found")
            print(f"Available assemblies: {[a.name for a in TEST_ASSEMBLIES]}")
            return False
    else:
        selected_assembly = TEST_ASSEMBLIES[0]

    print(f"[PROFILING] Profiling: {selected_assembly.name}")
    print(f"   URL: {selected_assembly.url}")
    print(f"   Expected parts: {selected_assembly.expected_parts}")
    print()

    profiler = WorkflowProfiler()

    try:
        profile = profiler.profile_assembly_workflow(selected_assembly.url, selected_assembly.name)

        if profile.success:
            print("\n[SUCCESS] Profiling completed successfully!")
            print(f"   Total time: {profile.total_duration:.2f}s")
            print(f"   Actual parts: {profile.complexity.part_count}")
            print(f"   Total API calls: {profile.api_call_summary['total_calls']}")
            print(f"   Successful API calls: {profile.api_call_summary['successful_calls']}")

            # Show phase breakdown
            print("\n[PROGRESS] Phase breakdown:")
            for phase in profile.phases:
                status = "[SUCCESS]" if phase.success else "[FAILED]"
                print(f"   {status} {phase.phase_name:<20}: {phase.duration:>8.3f}s ({phase.api_calls:>3} API calls)")

            # Show API call patterns
            print("\n[API] API call patterns:")
            calls_by_endpoint = profile.api_call_summary.get("calls_by_endpoint", {})
            for endpoint, count in sorted(calls_by_endpoint.items(), key=lambda x: x[1], reverse=True):
                endpoint_short = endpoint.replace("Client.", "").split("(")[0]  # Simplify display
                print(f"   {endpoint_short:<25}: {count:>3} calls")

            # Save results
            result_file = profiler.save_results(f"profile_{selected_assembly.name}.json")
            print(f"\n[SAVED] Results saved to: {result_file}")

            return True

        else:
            print("\n[FAILED] Profiling failed!")
            for error in profile.error_messages:
                print(f"   Error: {error}")
            return False

    except KeyboardInterrupt:
        print("\n[WARNING]  Profiling interrupted by user")
        return False
    except Exception as e:
        print(f"\n[FAILED] Profiling failed with exception: {e}")
        return False


def run_batch_profiling():
    """Profile all available assemblies"""
    if not TEST_ASSEMBLIES:
        print("[FAILED] No test assemblies configured!")
        return

    print(f"[BATCH] Batch profiling {len(TEST_ASSEMBLIES)} assemblies...")
    print("=" * 50)

    profiler = WorkflowProfiler()
    successful = 0
    failed = 0

    for assembly in TEST_ASSEMBLIES:
        print(f"\n[PROFILING] Profiling: {assembly.name}")

        try:
            profile = profiler.profile_assembly_workflow(assembly.url, assembly.name)

            if profile.success:
                successful += 1
                duration = profile.total_duration
                parts = profile.complexity.part_count
                api_calls = profile.api_call_summary["total_calls"]
                print(f"   [SUCCESS] Success: {duration:.2f}s, {parts} parts, {api_calls} API calls")
            else:
                failed += 1
                print(f"   [FAILED] Failed: {', '.join(profile.error_messages[:2])}")  # Show first 2 errors

        except Exception as e:
            failed += 1
            print(f"   [FAILED] Exception: {e}")

    print("\n[ANALYSIS] Batch profiling complete!")
    print(f"   Successful: {successful}/{len(TEST_ASSEMBLIES)}")
    print(f"   Failed: {failed}/{len(TEST_ASSEMBLIES)}")

    if successful > 0:
        # Generate summary
        summary = profiler.get_timing_summary()
        result_file = profiler.save_results("batch_profiling_results.json")

        print("\n[PROGRESS] Summary statistics:")
        print(f"   Total profiles: {summary['total_profiles']}")
        print(f"   Success rate: {summary['successful_profiles']}/{summary['total_profiles']}")

        phase_stats = summary.get("phase_stats", {})
        if phase_stats:
            print("\n[TIMING]  Average phase timings:")
            for phase_name, stats in phase_stats.items():
                avg_time = stats["avg_time"]
                avg_calls = stats["avg_api_calls"]
                print(f"   {phase_name:<20}: {avg_time:>8.3f}s ({avg_calls:>5.1f} API calls avg)")

        print(f"\n[SAVED] Results saved to: {result_file}")


def main():
    """Main execution"""
    import argparse

    parser = argparse.ArgumentParser(description="Run profiling on Onshape assemblies")
    parser.add_argument("--assembly", "-a", help="Specific assembly name to profile")
    parser.add_argument("--batch", "-b", action="store_true", help="Profile all assemblies")
    parser.add_argument("--list", "-l", action="store_true", help="List available assemblies")

    args = parser.parse_args()

    if args.list:
        print("Available assemblies:")
        for assembly in TEST_ASSEMBLIES:
            print(f"  • {assembly.name}: {assembly.expected_parts} parts ({assembly.complexity})")
        return

    if args.batch:
        run_batch_profiling()
    else:
        run_single_assembly_profile(args.assembly)


if __name__ == "__main__":
    main()
