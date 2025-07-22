#!/usr/bin/env python3
"""
Plotting utilities for profiling results

Visualize timing vs complexity relationships and API call patterns.
"""

import json
import statistics
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt


def load_profiling_results(results_dir: str | Path = "profiling_results") -> list[dict[str, Any]]:
    """Load all profiling result JSON files from directory"""
    results_dir = Path(results_dir)

    if not results_dir.exists():
        raise FileNotFoundError(f"Results directory not found: {results_dir}")

    results = []
    for json_file in results_dir.glob("*.json"):
        try:
            with json_file.open() as f:
                data = json.load(f)
                # Handle both single profile and multi-profile formats
                if isinstance(data, list):
                    results.extend(data)
                elif "raw_profiles" in data:
                    # Handle baseline analysis format with raw_profiles array
                    results.extend(data["raw_profiles"])
                elif "profiles" in data:
                    # Handle workflow profiler format with profiles array
                    results.extend(data["profiles"])
                else:
                    results.append(data)
        except (json.JSONDecodeError, KeyError) as e:
            print(f"Warning: Could not load {json_file}: {e}")

    return results


def plot_phase_timing_vs_complexity(results: list[dict[str, Any]], output_file: str = "phase_timing_vs_complexity.png"):
    """Plot phase timing vs assembly complexity"""
    if not results:
        print("No results to plot")
        return

    # Extract data for plotting
    complexity_data = []
    for result in results:
        if not result.get("success", False):
            continue

        complexity = result.get("complexity", {})
        part_count = complexity.get("part_count", 0)
        total_time = result.get("total_duration", 0)

        phases = result.get("phases", [])
        for phase in phases:
            if phase.get("success", False):
                complexity_data.append({
                    "assembly_name": result.get("assembly_name", "unknown"),
                    "part_count": part_count,
                    "phase_name": phase.get("phase_name", "unknown"),
                    "duration": phase.get("duration", 0),
                    "api_calls": phase.get("api_calls", 0),
                    "total_time": total_time,
                })

    if not complexity_data:
        print("No valid phase data found")
        return

    # Create subplots for different phases
    phases = list({d["phase_name"] for d in complexity_data})
    phases.sort()

    fig, axes = plt.subplots(2, (len(phases) + 1) // 2, figsize=(15, 10))
    fig.suptitle("Phase Timing vs Assembly Complexity (Part Count)", fontsize=16)

    axes = axes.flatten() if len(phases) > 1 else [axes]

    for i, phase in enumerate(phases):
        if i >= len(axes):
            break

        phase_data = [d for d in complexity_data if d["phase_name"] == phase]

        if not phase_data:
            continue

        part_counts = [d["part_count"] for d in phase_data]
        durations = [d["duration"] for d in phase_data]

        axes[i].scatter(part_counts, durations, alpha=0.6, s=30)
        axes[i].set_xlabel("Part Count")
        axes[i].set_ylabel("Duration (s)")
        axes[i].set_title(f"{phase.replace('_', ' ').title()}")
        axes[i].grid(True, alpha=0.3)

    # Hide unused subplots
    for i in range(len(phases), len(axes)):
        axes[i].set_visible(False)

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches="tight")
    print(f"Saved plot: {output_file}")
    plt.close()


def plot_api_calls_vs_complexity(results: list[dict[str, Any]], output_file: str = "api_calls_vs_complexity.png"):
    """Plot API calls vs assembly complexity"""
    if not results:
        print("No results to plot")
        return

    # Extract data
    plot_data = []
    for result in results:
        if not result.get("success", False):
            continue

        complexity = result.get("complexity", {})
        part_count = complexity.get("part_count", 0)
        api_summary = result.get("api_call_summary", {})
        total_calls = api_summary.get("total_calls", 0)

        plot_data.append({
            "assembly_name": result.get("assembly_name", "unknown"),
            "part_count": part_count,
            "total_api_calls": total_calls,
            "total_time": result.get("total_duration", 0),
        })

    if not plot_data:
        print("No valid API call data found")
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle("API Usage vs Assembly Complexity", fontsize=16)

    # Plot 1: API calls vs part count
    part_counts = [d["part_count"] for d in plot_data]
    api_calls = [d["total_api_calls"] for d in plot_data]

    ax1.scatter(part_counts, api_calls, alpha=0.7, s=50)
    ax1.set_xlabel("Part Count")
    ax1.set_ylabel("Total API Calls")
    ax1.set_title("API Calls vs Part Count")
    ax1.grid(True, alpha=0.3)

    # Plot 2: API calls vs total time
    total_times = [d["total_time"] for d in plot_data]

    ax2.scatter(total_times, api_calls, alpha=0.7, s=50, color="orange")
    ax2.set_xlabel("Total Time (s)")
    ax2.set_ylabel("Total API Calls")
    ax2.set_title("API Calls vs Total Time")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches="tight")
    print(f"Saved plot: {output_file}")
    plt.close()


def plot_phase_breakdown(results: list[dict[str, Any]], output_file: str = "phase_breakdown.png"):
    """Plot average phase breakdown across all assemblies"""
    if not results:
        print("No results to plot")
        return

    # Aggregate phase data
    phase_stats = {}
    for result in results:
        if not result.get("success", False):
            continue

        phases = result.get("phases", [])
        for phase in phases:
            phase_name = phase.get("phase_name", "unknown")
            if phase_name not in phase_stats:
                phase_stats[phase_name] = {"times": [], "api_calls": []}

            if phase.get("success", False):
                phase_stats[phase_name]["times"].append(phase.get("duration", 0))
                phase_stats[phase_name]["api_calls"].append(phase.get("api_calls", 0))

    if not phase_stats:
        print("No valid phase data found")
        return

    # Calculate averages
    phase_names = []
    avg_times = []
    avg_api_calls = []

    for phase_name, stats in phase_stats.items():
        if stats["times"]:
            phase_names.append(phase_name.replace("_", " ").title())
            avg_times.append(statistics.mean(stats["times"]))
            avg_api_calls.append(statistics.mean(stats["api_calls"]))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    fig.suptitle("Average Phase Performance", fontsize=16)

    # Plot 1: Average timing
    bars1 = ax1.bar(phase_names, avg_times, color="skyblue", alpha=0.8)
    ax1.set_ylabel("Average Duration (s)")
    ax1.set_title("Average Phase Duration")
    ax1.tick_params(axis="x", rotation=45)

    # Add value labels on bars
    for bar, value in zip(bars1, avg_times, strict=False):
        ax1.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.01,
            f"{value:.2f}s",
            ha="center",
            va="bottom",
            fontsize=9,
        )

    # Plot 2: Average API calls
    bars2 = ax2.bar(phase_names, avg_api_calls, color="lightcoral", alpha=0.8)
    ax2.set_ylabel("Average API Calls")
    ax2.set_title("Average API Calls per Phase")
    ax2.tick_params(axis="x", rotation=45)

    # Add value labels on bars
    for bar, value in zip(bars2, avg_api_calls, strict=False):
        ax2.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.1,
            f"{value:.1f}",
            ha="center",
            va="bottom",
            fontsize=9,
        )

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches="tight")
    print(f"Saved plot: {output_file}")
    plt.close()


def generate_performance_report(results: list[dict[str, Any]], output_file: str = "performance_report.txt"):
    """Generate text performance report"""
    if not results:
        print("No results to analyze")
        return

    successful_results = [r for r in results if r.get("success", False)]

    report_lines = [
        "PERFORMANCE ANALYSIS REPORT",
        "=" * 50,
        f"Total profiles analyzed: {len(results)}",
        f"Successful profiles: {len(successful_results)}",
        f"Success rate: {len(successful_results) / len(results) * 100:.1f}%",
        "",
        "ASSEMBLY COMPLEXITY DISTRIBUTION:",
    ]

    # Complexity analysis
    part_counts = [r.get("complexity", {}).get("part_count", 0) for r in successful_results]
    if part_counts:
        report_lines.extend([
            f"  Min parts: {min(part_counts)}",
            f"  Max parts: {max(part_counts)}",
            f"  Avg parts: {statistics.mean(part_counts):.1f}",
            f"  Median parts: {statistics.median(part_counts):.1f}",
            "",
        ])

    # Timing analysis
    total_times = [r.get("total_duration", 0) for r in successful_results]
    if total_times:
        report_lines.extend([
            "TIMING ANALYSIS:",
            f"  Min total time: {min(total_times):.2f}s",
            f"  Max total time: {max(total_times):.2f}s",
            f"  Avg total time: {statistics.mean(total_times):.2f}s",
            f"  Median total time: {statistics.median(total_times):.2f}s",
            "",
        ])

    # API call analysis
    api_calls = [r.get("api_call_summary", {}).get("total_calls", 0) for r in successful_results]
    if api_calls:
        report_lines.extend([
            "API CALL ANALYSIS:",
            f"  Min API calls: {min(api_calls)}",
            f"  Max API calls: {max(api_calls)}",
            f"  Avg API calls: {statistics.mean(api_calls):.1f}",
            f"  Median API calls: {statistics.median(api_calls):.1f}",
            "",
        ])

    # Phase analysis
    phase_stats = {}
    for result in successful_results:
        for phase in result.get("phases", []):
            if phase.get("success", False):
                phase_name = phase.get("phase_name", "unknown")
                if phase_name not in phase_stats:
                    phase_stats[phase_name] = []
                phase_stats[phase_name].append(phase.get("duration", 0))

    if phase_stats:
        report_lines.extend(["PHASE TIMING ANALYSIS:", f"{'Phase':<20} {'Avg Time':<12} {'Max Time':<12} {'Samples'}"])
        for phase_name in sorted(phase_stats.keys()):
            times = phase_stats[phase_name]
            avg_time = statistics.mean(times)
            max_time = max(times)
            sample_count = len(times)

            formatted_name = phase_name.replace("_", " ").title()[:19]
            report_lines.append(f"{formatted_name:<20} {avg_time:<12.3f} {max_time:<12.3f} {sample_count}")

    # Write report
    with open(output_file, "w") as f:
        f.write("\n".join(report_lines))

    print(f"Generated report: {output_file}")


def main():
    """Generate all plots and reports"""
    import argparse

    parser = argparse.ArgumentParser(description="Generate profiling visualizations")
    parser.add_argument("--results-dir", default="profiling_results", help="Directory containing profiling JSON files")
    parser.add_argument("--output-dir", default="plots", help="Directory to save plots")

    args = parser.parse_args()

    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(exist_ok=True)

    # Load results
    print(f"Loading results from {args.results_dir}...")
    try:
        results = load_profiling_results(args.results_dir)
        print(f"Loaded {len(results)} profiling results")
    except FileNotFoundError as e:
        print(f"Error: {e}")
        return

    if not results:
        print("No results found to plot")
        return

    # Generate plots
    print("Generating plots...")
    plot_phase_timing_vs_complexity(results, output_dir / "phase_timing_vs_complexity.png")
    plot_api_calls_vs_complexity(results, output_dir / "api_calls_vs_complexity.png")
    plot_phase_breakdown(results, output_dir / "phase_breakdown.png")

    # Generate report
    print("Generating performance report...")
    generate_performance_report(results, output_dir / "performance_report.txt")

    print(f"All outputs saved to {output_dir}")


if __name__ == "__main__":
    main()
