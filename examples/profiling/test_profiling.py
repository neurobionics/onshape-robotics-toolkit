#!/usr/bin/env python3
"""
Quick test of the profiling framework without running full analysis
"""

import sys
from pathlib import Path

# Add the toolkit to path
sys.path.append(str(Path(__file__).parent.parent.parent / "src"))

import time

from test_assemblies import TEST_ASSEMBLIES
from workflow_profiler import WorkflowProfiler


def test_profiling_framework():
    """Test that profiling framework works without full analysis"""
    print("🧪 Testing Profiling Framework")
    print("=" * 50)

    # Test basic profiler initialization
    profiler = WorkflowProfiler()
    print("[SUCCESS] Profiler initialized successfully")

    # Test phase profiling with a dummy function
    def dummy_phase():
        """Simulate a workflow phase"""
        time.sleep(0.1)  # Simulate some work
        return {"test": True, "duration": 0.1}

    metrics = profiler._profile_phase("test_phase", dummy_phase)
    print(f"[SUCCESS] Phase profiling works: {metrics.phase_name} took {metrics.duration:.3f}s")

    # Test analysis without real data
    if len(profiler.results) == 0:
        print("[SUCCESS] No results yet (expected)")

    # Test configuration
    if TEST_ASSEMBLIES:
        print(f"[SUCCESS] Found {len(TEST_ASSEMBLIES)} test assemblies configured")
        for assembly in TEST_ASSEMBLIES:
            print(f"   • {assembly.name} ({assembly.complexity})")
    else:
        print("[WARNING]  No test assemblies configured - add some to test_assemblies.py")

    print("\n[TARGET] Framework ready for baseline analysis!")
    print("Next step: Run 'uv run python run_baseline_analysis.py'")


if __name__ == "__main__":
    test_profiling_framework()
