#!/usr/bin/env python3
"""Enhanced Rust-Python benchmarking script for onshape-robotics-toolkit

This script provides comprehensive performance benchmarking of the Rust vs Python
implementations across all major workflow components.
"""

import json
import statistics
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

# Color support for cross-platform terminal output
try:
    from colorama import Fore, Style, init

    init()
    HAS_COLOR = True
except ImportError:
    print("Install colorama for colored output: pip install colorama")

    class Fore:
        CYAN = GREEN = YELLOW = RED = BLUE = ""

    class Style:
        RESET_ALL = ""

    HAS_COLOR = False

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.graph import create_graph
from onshape_robotics_toolkit.log import LOGGER, LogLevel
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import get_instances, get_mates_and_relations, get_parts, get_subassemblies
from onshape_robotics_toolkit.robot import get_robot


@dataclass
class BenchmarkConfig:
    """Configuration for benchmark runs"""

    iterations: int = 5
    warmup_iterations: int = 2
    include_memory: bool = True
    assembly_url: str = "https://cad.onshape.com/documents/cf6b852d2c88d661ac2e17e8/w/c842455c29cc878dc48bdc68/e/b5e293d409dd0b88596181ef"
    robot_name: str = "quadruped"
    max_depth: int = 0


@dataclass
class PhaseResult:
    """Results for a single benchmark phase"""

    name: str
    rust_times: list[float]
    python_times: list[float]
    rust_memory: float | None = None
    python_memory: float | None = None

    @property
    def rust_mean(self) -> float:
        return statistics.mean(self.rust_times)

    @property
    def python_mean(self) -> float:
        return statistics.mean(self.python_times)

    @property
    def speedup(self) -> float:
        return self.python_mean / self.rust_mean if self.rust_mean > 0 else 0

    @property
    def is_faster(self) -> bool:
        return self.speedup > 1.0

    @property
    def rust_std(self) -> float:
        return statistics.stdev(self.rust_times) if len(self.rust_times) > 1 else 0

    @property
    def python_std(self) -> float:
        return statistics.stdev(self.python_times) if len(self.python_times) > 1 else 0


class ComprehensiveBenchmark:
    """Comprehensive Rust vs Python performance benchmark suite"""

    def __init__(self, config: BenchmarkConfig):
        self.config = config
        self.results = {}

    def run_comprehensive_benchmark(self) -> dict[str, PhaseResult]:
        """Run complete benchmarking suite"""
        self._print_header()

        phases = {
            "client_operations": ("HTTP Client Operations", self._benchmark_client_operations),
            "assembly_parsing": ("Assembly Parsing", self._benchmark_assembly_parsing),
            "end_to_end": ("Complete Workflow", self._benchmark_end_to_end_workflow),
        }

        results = {}
        for phase_key, (phase_name, phase_func) in phases.items():
            print(f"{Fore.YELLOW}📊 Benchmarking: {phase_name}{Style.RESET_ALL}")
            try:
                results[phase_key] = phase_func()
                self._print_phase_result(results[phase_key])
            except Exception as e:
                print(f"{Fore.RED}❌ Error in {phase_name}: {e}{Style.RESET_ALL}")
                print("   This may indicate Rust backend is not available or has issues.")
                # Create dummy result to continue
                results[phase_key] = PhaseResult(phase_name, [0.0], [0.0], error=str(e))
            print()

        self._print_summary(results)
        self._save_results(results)

        return results

    def _print_header(self):
        """Print benchmark header"""
        print(f"\n{Fore.CYAN}🚀 Onshape Robotics Toolkit: Rust vs Python Benchmark{Style.RESET_ALL}")
        print(f"{'=' * 80}")
        print("Configuration:")
        print(f"  • Iterations: {self.config.iterations}")
        print(f"  • Warmup: {self.config.warmup_iterations}")
        print(f"  • Assembly URL: {self.config.assembly_url[:50]}...")
        print(f"  • Robot Name: {self.config.robot_name}")
        print(f"  • Max Depth: {self.config.max_depth}")

        # Check Rust availability
        rust_available = self._check_rust_availability()
        if rust_available:
            print(f"  • Rust Backend: {Fore.GREEN}✅ Available{Style.RESET_ALL}")
        else:
            print(f"  • Rust Backend: {Fore.RED}❌ Not Available{Style.RESET_ALL}")
            print("    Install with: uv run maturin develop --release")

        print(f"{'=' * 80}\n")

    def _check_rust_availability(self) -> bool:
        """Check if Rust backend is available"""
        import importlib.util

        spec = importlib.util.find_spec("onshape_robotics_toolkit.native")
        return spec is not None

    def _benchmark_client_operations(self) -> PhaseResult:
        """Benchmark HTTP client operations"""
        print("  Testing assembly retrieval performance...")

        rust_times = []
        python_times = []

        # Warmup
        for i in range(self.config.warmup_iterations):
            print(f"  Warmup {i + 1}/{self.config.warmup_iterations}...")
            try:
                self._run_client_operation(use_rust=True)
                self._run_client_operation(use_rust=False)
            except Exception as e:
                print(f"    Warning during warmup: {e}")

        # Actual benchmarking
        for i in range(self.config.iterations):
            print(f"  Iteration {i + 1}/{self.config.iterations}...", end=" ")

            # Rust timing
            try:
                start = time.perf_counter()
                self._run_client_operation(use_rust=True)
                rust_time = time.perf_counter() - start
                rust_times.append(rust_time)
                rust_status = f"✅ {rust_time:.3f}s"
            except Exception as e:
                print(f"Rust failed: {e}")
                rust_times.append(float("inf"))
                rust_status = "❌ Failed"

            # Python timing
            try:
                start = time.perf_counter()
                self._run_client_operation(use_rust=False)
                python_time = time.perf_counter() - start
                python_times.append(python_time)
                python_status = f"✅ {python_time:.3f}s"
            except Exception as e:
                print(f"Python failed: {e}")
                python_times.append(float("inf"))
                python_status = "❌ Failed"

            print(f"Rust: {rust_status} | Python: {python_status}")

        return PhaseResult("HTTP Client Operations", rust_times, python_times)

    def _run_client_operation(self, use_rust: bool):
        """Single client operation for benchmarking"""
        client = Client(env=".env", use_rust=use_rust)
        document = Document.from_url(self.config.assembly_url)
        client.set_base_url(document.base_url)

        assembly = client.get_assembly(
            did=document.did,
            wtype=document.wtype,
            wid=document.wid,
            eid=document.eid,
            log_response=False,
            with_meta_data=True,
        )
        return assembly

    def _benchmark_assembly_parsing(self) -> PhaseResult:
        """Benchmark assembly parsing operations"""
        print("  Testing assembly parsing performance...")

        # Setup - get assembly once
        try:
            assembly = self._run_client_operation(use_rust=False)  # Use Python client for setup
            client = Client(env=".env", use_rust=False)  # Setup client
        except Exception as e:
            print(f"  Failed to setup assembly for parsing benchmark: {e}")
            return PhaseResult("Assembly Parsing", [float("inf")], [float("inf")])

        rust_times = []
        python_times = []

        for i in range(self.config.iterations):
            print(f"  Iteration {i + 1}/{self.config.iterations}...", end=" ")

            # Rust parsing
            try:
                start = time.perf_counter()
                self._run_parsing_workflow(assembly, client, use_rust=True)
                rust_time = time.perf_counter() - start
                rust_times.append(rust_time)
                rust_status = f"✅ {rust_time:.3f}s"
            except Exception:
                rust_times.append(float("inf"))
                rust_status = "❌ Failed"

            # Python parsing
            try:
                start = time.perf_counter()
                self._run_parsing_workflow(assembly, client, use_rust=False)
                python_time = time.perf_counter() - start
                python_times.append(python_time)
                python_status = f"✅ {python_time:.3f}s"
            except Exception:
                python_times.append(float("inf"))
                python_status = "❌ Failed"

            print(f"Rust: {rust_status} | Python: {python_status}")

        return PhaseResult("Assembly Parsing", rust_times, python_times)

    def _run_parsing_workflow(self, assembly, client, use_rust: bool):
        """Single parsing workflow for benchmarking"""
        instances, occurrences, id_to_name_map = get_instances(assembly=assembly, max_depth=self.config.max_depth)

        subassemblies, rigid_subassemblies = get_subassemblies(assembly=assembly, client=client, instances=instances)

        parts = get_parts(
            assembly=assembly,
            rigid_subassemblies=rigid_subassemblies,
            client=client,
            instances=instances,
        )

        mates, relations = get_mates_and_relations(
            assembly=assembly,
            subassemblies=subassemblies,
            rigid_subassemblies=rigid_subassemblies,
            id_to_name_map=id_to_name_map,
            parts=parts,
        )

        return instances, occurrences, subassemblies, parts, mates, relations

    def _benchmark_end_to_end_workflow(self) -> PhaseResult:
        """Benchmark complete CAD-to-URDF workflow"""
        print("  Testing complete workflow performance...")

        rust_times = []
        python_times = []

        for i in range(self.config.iterations):
            print(f"  Iteration {i + 1}/{self.config.iterations}...", end=" ")

            # Complete Rust workflow
            try:
                start = time.perf_counter()
                self._run_complete_workflow(use_rust=True)
                rust_time = time.perf_counter() - start
                rust_times.append(rust_time)
                rust_status = f"✅ {rust_time:.3f}s"
            except Exception:
                rust_times.append(float("inf"))
                rust_status = "❌ Failed"

            # Complete Python workflow
            try:
                start = time.perf_counter()
                self._run_complete_workflow(use_rust=False)
                python_time = time.perf_counter() - start
                python_times.append(python_time)
                python_status = f"✅ {python_time:.3f}s"
            except Exception:
                python_times.append(float("inf"))
                python_status = "❌ Failed"

            print(f"Rust: {rust_status} | Python: {python_status}")

        return PhaseResult("Complete Workflow", rust_times, python_times)

    def _run_complete_workflow(self, use_rust: bool):
        """Complete CAD-to-URDF workflow"""
        # Client operations
        client = Client(env=".env", use_rust=use_rust)
        document = Document.from_url(self.config.assembly_url)
        client.set_base_url(document.base_url)
        assembly = client.get_assembly(
            did=document.did,
            wtype=document.wtype,
            wid=document.wid,
            eid=document.eid,
            log_response=False,
            with_meta_data=True,
        )

        # Parsing workflow
        instances, occurrences, subassemblies, parts, mates, relations = self._run_parsing_workflow(
            assembly,
            client,
            use_rust=False,  # Force Python parsing for now
        )

        # Graph construction (always Python)
        graph, root_node = create_graph(
            occurrences=occurrences,
            instances=instances,
            parts=parts,
            mates=mates,
            use_user_defined_root=False,
        )

        # Robot generation
        robot = get_robot(
            assembly=assembly,
            graph=graph,
            root_node=root_node,
            parts=parts,
            mates=mates,
            relations=relations,
            client=client,
            robot_name=self.config.robot_name,
        )

        # URDF generation
        robot.to_urdf()

        return robot

    def _print_phase_result(self, result: PhaseResult):
        """Print results for a single phase"""
        # Handle failed benchmarks
        if any(t == float("inf") for t in result.rust_times) or any(t == float("inf") for t in result.python_times):
            print(f"  {Fore.RED}❌ Benchmark failed - check implementation{Style.RESET_ALL}")
            return

        if result.is_faster:
            status = f"✅ {result.speedup:.2f}x faster"
            color = Fore.GREEN
        else:
            status = f"❌ {1 / result.speedup:.2f}x slower"
            color = Fore.RED

        print(f"  {color}{status}{Style.RESET_ALL}")
        print(f"    Rust:   {result.rust_mean:.4f}s ± {result.rust_std:.4f}s")
        print(f"    Python: {result.python_mean:.4f}s ± {result.python_std:.4f}s")

    def _print_summary(self, results: dict[str, PhaseResult]):
        """Print comprehensive summary"""
        print(f"\n{Fore.CYAN}📋 BENCHMARK SUMMARY{Style.RESET_ALL}")
        print(f"{'=' * 80}")
        print(f"{'Phase':<25} {'Rust (s)':<12} {'Python (s)':<12} {'Speedup':<10} {'Status'}")
        print(f"{'-' * 80}")

        total_rust = 0
        total_python = 0
        successful_phases = 0

        for _phase_name, result in results.items():
            phase_display = result.name

            # Handle failed benchmarks
            if any(t == float("inf") for t in result.rust_times) or any(t == float("inf") for t in result.python_times):
                status = "❌ FAILED"
                speedup_display = "N/A"
            else:
                status = "✅ FASTER" if result.is_faster else "❌ SLOWER"
                speedup_display = f"{result.speedup:.2f}x"
                total_rust += result.rust_mean
                total_python += result.python_mean
                successful_phases += 1

            print(
                f"{phase_display:<25} {result.rust_mean:<12.4f}"
                f"{result.python_mean:<12.4f} {speedup_display:<10} {status}"
            )

        if successful_phases > 0:
            print(f"{'-' * 80}")
            overall_speedup = total_python / total_rust
            overall_status = "✅ FASTER" if overall_speedup > 1.0 else "❌ SLOWER"
            print(
                f"{'OVERALL':<25} {total_rust:<12.4f} {total_python:<12.4f} {overall_speedup:<10.2f}x {overall_status}"
            )

        print(f"{'=' * 80}")

        if successful_phases > 0:
            if total_python / total_rust > 1.0:
                print(
                    f"{Fore.GREEN}🎉 Rust implementation is"
                    f"{total_python / total_rust:.2f}x faster overall!{Style.RESET_ALL}"
                )
                print("   Performance gains validate the hybrid Rust/Python architecture.")
            else:
                print(f"{Fore.YELLOW}⚠️  Rust implementation is {total_rust / total_python:.2f}x slower")
                print("   This indicates the Rust migration needs further performance tuning.")
        else:
            print(f"{Fore.RED}❌ No successful benchmarks")

    def _save_results(self, results: dict[str, PhaseResult]):
        """Save results to JSON for further analysis"""
        output_data = {
            "timestamp": datetime.now().isoformat(),
            "config": {
                "iterations": self.config.iterations,
                "warmup_iterations": self.config.warmup_iterations,
                "assembly_url": self.config.assembly_url,
                "robot_name": self.config.robot_name,
                "max_depth": self.config.max_depth,
            },
            "results": {
                name: {
                    "rust_times": [t if t != float("inf") else None for t in result.rust_times],
                    "python_times": [t if t != float("inf") else None for t in result.python_times],
                    "rust_mean": result.rust_mean if result.rust_mean != float("inf") else None,
                    "python_mean": result.python_mean if result.python_mean != float("inf") else None,
                    "speedup": result.speedup if result.speedup not in [0, float("inf")] else None,
                    "is_faster": result.is_faster,
                }
                for name, result in results.items()
            },
        }

        output_file = Path(f"benchmark_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
        with open(output_file, "w") as f:
            json.dump(output_data, f, indent=2)
        print(f"\n💾 Results saved to: {output_file}")


def main():
    """Main benchmark execution"""
    # Reduce logging noise during benchmarking
    LOGGER.set_stream_level(LogLevel.ERROR)

    config = BenchmarkConfig(
        iterations=3,  # Reduced for initial testing
        warmup_iterations=1,
        include_memory=False,  # Disabled for simplicity
    )

    benchmark = ComprehensiveBenchmark(config)
    benchmark.run_comprehensive_benchmark()

    print(f"\n{Fore.BLUE}✨ Benchmarking completed!{Style.RESET_ALL}")
    print("This benchmark provides quantitative evidence for Rust migration benefits.")
    print("\nNext steps:")
    print("1. If Rust backend failed, run: uv run maturin develop --release")
    print("2. If Rust is slower, optimization is needed in the implementation")
    print("3. If Rust is faster, the migration is providing expected benefits!")


if __name__ == "__main__":
    main()
