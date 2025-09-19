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
import numpy as np


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


def plot_phase_timing_vs_complexity(
    results: list[dict[str, Any]],
    output_file: str = "phase_timing_vs_complexity.png",
    complexity_metric: str = "occurrence_count",
):
    """Plot phase timing vs assembly complexity

    Args:
        results: List of profiling results
        output_file: Output filename
        complexity_metric: Metric to use for complexity (occurrence_count, part_count, or robot_links)
    """
    if not results:
        print("No results to plot")
        return

    # Extract data for plotting
    complexity_data = []
    for result in results:
        if not result.get("success", False):
            continue

        complexity = result.get("complexity", {})
        complexity_value = complexity.get(complexity_metric, 0)
        # Fallback to part_count if preferred metric is not available
        if complexity_value == 0 and complexity_metric != "part_count":
            complexity_value = complexity.get("part_count", 0)

        total_time = result.get("total_duration", 0)

        phases = result.get("phases", [])
        for phase in phases:
            if phase.get("success", False):
                complexity_data.append({
                    "assembly_name": result.get("assembly_name", "unknown"),
                    "complexity_value": complexity_value,
                    "phase_name": phase.get("phase_name", "unknown"),
                    "duration": phase.get("duration", 0),
                    "api_calls": phase.get("api_calls", 0),
                    "total_time": total_time,
                })

    if not complexity_data:
        print("No valid phase data found")
        return

    # Get unique phases and focus on the most time-consuming ones
    phases = list({d["phase_name"] for d in complexity_data})

    # Calculate average duration per phase to identify most important ones
    phase_avg_duration = {}
    for phase in phases:
        phase_data = [d["duration"] for d in complexity_data if d["phase_name"] == phase]
        phase_avg_duration[phase] = sum(phase_data) / len(phase_data) if phase_data else 0

    # Sort phases by average duration (descending) and take top 6 to reduce crowding
    top_phases = sorted(phases, key=lambda p: phase_avg_duration[p], reverse=True)[:6]

    # Create a more readable layout: 2x3 grid for top 6 phases
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))

    # Use more user-friendly display names
    metric_display_names = {
        "part_count": "Number of Parts",
        "occurrence_count": "Number of Parts",
        "robot_links": "Robot Links",
    }
    metric_display_name = metric_display_names.get(complexity_metric, complexity_metric.replace("_", " ").title())
    fig.suptitle(f"Phase Timing vs Assembly Complexity ({metric_display_name}) - Top 6 Phases", fontsize=16)

    axes = axes.flatten()
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]

    for i, phase in enumerate(top_phases):
        phase_data = [d for d in complexity_data if d["phase_name"] == phase]

        if not phase_data:
            continue

        complexity_values = [d["complexity_value"] for d in phase_data]
        durations = [d["duration"] for d in phase_data]

        axes[i].scatter(complexity_values, durations, alpha=0.7, s=60, color=colors[i])
        axes[i].set_xlabel(metric_display_name)
        axes[i].set_ylabel("Duration (s)")
        axes[i].set_title(f"{phase.replace('_', ' ').title()}\n(avg: {phase_avg_duration[phase]:.3f}s)")
        axes[i].grid(True, alpha=0.3)

        # Format x-axis to show integer values for part counts
        if complexity_metric in ["part_count", "occurrence_count", "robot_links"]:
            axes[i].xaxis.set_major_locator(plt.MaxNLocator(integer=True))

        # Add trend line if we have enough data points
        if len(complexity_values) > 2:
            z = np.polyfit(complexity_values, durations, 1)
            p = np.poly1d(z)
            axes[i].plot(sorted(complexity_values), p(sorted(complexity_values)), "--", alpha=0.8, color=colors[i])

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches="tight")
    print(f"Saved plot: {output_file} (showing top 6 phases by average duration)")
    plt.close()


def plot_api_calls_vs_complexity(
    results: list[dict[str, Any]],
    output_file: str = "api_calls_vs_complexity.png",
    complexity_metric: str = "occurrence_count",
):
    """Plot API calls vs assembly complexity

    Args:
        results: List of profiling results
        output_file: Output filename
        complexity_metric: Metric to use for complexity (occurrence_count, part_count, or robot_links)
    """
    if not results:
        print("No results to plot")
        return

    # Extract data
    plot_data = []
    for result in results:
        if not result.get("success", False):
            continue

        complexity = result.get("complexity", {})
        complexity_value = complexity.get(complexity_metric, 0)
        # Fallback to part_count if preferred metric is not available
        if complexity_value == 0 and complexity_metric != "part_count":
            complexity_value = complexity.get("part_count", 0)

        api_summary = result.get("api_call_summary", {})
        total_calls = api_summary.get("total_calls", 0)

        plot_data.append({
            "assembly_name": result.get("assembly_name", "unknown"),
            "complexity_value": complexity_value,
            "total_api_calls": total_calls,
            "total_time": result.get("total_duration", 0),
        })

    if not plot_data:
        print("No valid API call data found")
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    # Use more user-friendly display names
    metric_display_names = {
        "part_count": "Number of Parts",
        "occurrence_count": "Number of Parts",
        "robot_links": "Robot Links",
    }
    metric_display_name = metric_display_names.get(complexity_metric, complexity_metric.replace("_", " ").title())
    fig.suptitle(f"API Usage vs Assembly Complexity ({metric_display_name})", fontsize=16)

    # Plot 1: API calls vs complexity metric
    complexity_values = [d["complexity_value"] for d in plot_data]
    api_calls = [d["total_api_calls"] for d in plot_data]

    ax1.scatter(complexity_values, api_calls, alpha=0.7, s=80, color="#2ca02c")
    ax1.set_xlabel(metric_display_name)
    ax1.set_ylabel("Total API Calls")
    ax1.set_title(f"API Calls vs {metric_display_name}")
    ax1.grid(True, alpha=0.3)

    # Format axes to show integer values
    if complexity_metric in ["part_count", "occurrence_count", "robot_links"]:
        ax1.xaxis.set_major_locator(plt.MaxNLocator(integer=True))
    ax1.yaxis.set_major_locator(plt.MaxNLocator(integer=True))

    # Add trend line if we have enough data points
    if len(complexity_values) > 2:
        z = np.polyfit(complexity_values, api_calls, 1)
        p = np.poly1d(z)
        ax1.plot(sorted(complexity_values), p(sorted(complexity_values)), "--", alpha=0.8, color="#2ca02c")

    # Plot 2: API calls vs total time
    total_times = [d["total_time"] for d in plot_data]

    ax2.scatter(total_times, api_calls, alpha=0.7, s=80, color="#ff7f0e")
    ax2.set_xlabel("Total Time (s)")
    ax2.set_ylabel("Total API Calls")
    ax2.set_title("API Calls vs Total Time")
    ax2.grid(True, alpha=0.3)

    # Format y-axis to show integer values for API calls
    ax2.yaxis.set_major_locator(plt.MaxNLocator(integer=True))

    # Add trend line if we have enough data points
    if len(total_times) > 2:
        z = np.polyfit(total_times, api_calls, 1)
        p = np.poly1d(z)
        ax2.plot(sorted(total_times), p(sorted(total_times)), "--", alpha=0.8, color="#ff7f0e")

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


def plot_endpoint_calls_breakdown(results: list[dict[str, Any]], output_file: str = "endpoint_calls_breakdown.png"):
    """Plot API calls breakdown by endpoint functions"""
    if not results:
        print("No results to plot")
        return

    # Aggregate endpoint data across all assemblies
    all_endpoints = {}
    for result in results:
        if not result.get("success", False):
            continue

        api_summary = result.get("api_call_summary", {})
        calls_by_endpoint = api_summary.get("calls_by_endpoint", {})

        for endpoint, count in calls_by_endpoint.items():
            if endpoint not in all_endpoints:
                all_endpoints[endpoint] = []
            all_endpoints[endpoint].append(count)

    if not all_endpoints:
        print("No endpoint data found in results")
        return

    # Calculate statistics for each endpoint
    endpoint_stats = {}
    for endpoint, counts in all_endpoints.items():
        endpoint_stats[endpoint] = {
            "total_calls": sum(counts),
            "avg_calls": statistics.mean(counts),
            "max_calls": max(counts),
            "frequency": len(counts),  # How many assemblies used this endpoint
        }

    # Sort endpoints by total calls (descending)
    sorted_endpoints = sorted(endpoint_stats.items(), key=lambda x: x[1]["total_calls"], reverse=True)

    # Take top 10-12 endpoints to avoid overcrowding
    top_endpoints = sorted_endpoints[:12]

    if not top_endpoints:
        print("No endpoint data to plot")
        return

    # Prepare data for plotting
    endpoint_names = []
    total_calls = []
    avg_calls = []

    for endpoint, stats in top_endpoints:
        # Clean up endpoint name for display - extract just the function name
        clean_name = endpoint.replace("Client.", "")
        # Remove document IDs but keep the function name clean
        if "(" in clean_name:
            clean_name = clean_name.split("(")[0]
        endpoint_names.append(clean_name)
        total_calls.append(stats["total_calls"])
        avg_calls.append(stats["avg_calls"])

    # Create the plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 10))
    fig.suptitle("API Endpoint Function Call Analysis", fontsize=16)

    # Plot 1: Total calls per endpoint
    bars1 = ax1.bar(endpoint_names, total_calls, color="steelblue", alpha=0.8)
    ax1.set_ylabel("Total API Calls")
    ax1.set_title("Total API Calls by Endpoint Function")
    ax1.set_xlabel("Endpoint Function")
    ax1.tick_params(axis="x", rotation=45)
    ax1.yaxis.set_major_locator(plt.MaxNLocator(integer=True))

    # Add value labels on bars
    for bar, value in zip(bars1, total_calls, strict=False):
        ax1.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + max(total_calls) * 0.01,
            f"{value}",
            ha="center",
            va="bottom",
            fontsize=10,
            weight="bold",
        )

    # Plot 2: Average calls per endpoint per assembly
    bars2 = ax2.bar(endpoint_names, avg_calls, color="darkorange", alpha=0.8)
    ax2.set_ylabel("Average API Calls per Assembly")
    ax2.set_title("Average API Calls per Assembly by Endpoint Function")
    ax2.set_xlabel("Endpoint Function")
    ax2.tick_params(axis="x", rotation=45)

    # Add value labels on bars
    for bar, value in zip(bars2, avg_calls, strict=False):
        ax2.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + max(avg_calls) * 0.01,
            f"{value:.1f}",
            ha="center",
            va="bottom",
            fontsize=10,
            weight="bold",
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

    # Complexity analysis - using occurrence_count as primary metric (displayed as "Number of Parts")
    occurrence_counts = [r.get("complexity", {}).get("occurrence_count", 0) for r in successful_results]
    [r.get("complexity", {}).get("part_count", 0) for r in successful_results]
    robot_links = [r.get("complexity", {}).get("robot_links", 0) for r in successful_results]

    if occurrence_counts and any(occurrence_counts):
        report_lines.extend([
            f"  Min parts: {min(occurrence_counts)}",
            f"  Max parts: {max(occurrence_counts)}",
            f"  Avg parts: {statistics.mean(occurrence_counts):.1f}",
            f"  Median parts: {statistics.median(occurrence_counts):.1f}",
            "",
        ])
    if robot_links and any(robot_links):
        report_lines.extend([
            f"  Min robot links: {min(robot_links)}",
            f"  Max robot links: {max(robot_links)}",
            f"  Avg robot links: {statistics.mean(robot_links):.1f}",
            f"  Median robot links: {statistics.median(robot_links):.1f}",
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
    parser.add_argument(
        "--complexity-metric",
        default="occurrence_count",
        choices=["occurrence_count", "part_count", "robot_links"],
        help="Metric to use for complexity analysis (default: occurrence_count)",
    )

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

    # Generate plots with the specified complexity metric
    print(f"Generating plots using {args.complexity_metric.replace('_', ' ')} as complexity metric...")
    plot_phase_timing_vs_complexity(results, output_dir / "phase_timing_vs_complexity.png", args.complexity_metric)
    plot_api_calls_vs_complexity(results, output_dir / "api_calls_vs_complexity.png", args.complexity_metric)
    plot_phase_breakdown(results, output_dir / "phase_breakdown.png")
    plot_endpoint_calls_breakdown(results, output_dir / "endpoint_calls_breakdown.png")

    # Generate report
    print("Generating performance report...")
    generate_performance_report(results, output_dir / "performance_report.txt")

    print(f"All outputs saved to {output_dir}")
    print(f"Complexity metric used: {args.complexity_metric.replace('_', ' ').title()}")


if __name__ == "__main__":
    main()
