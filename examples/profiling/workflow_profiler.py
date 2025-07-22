#!/usr/bin/env python3
"""
Comprehensive Workflow Profiler for Onshape Robotics Toolkit

This module provides detailed performance profiling of the CAD-to-URDF workflow
to identify real bottlenecks and validate optimization targets.
"""

import json
import statistics
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

# Import API tracking
from api_call_tracker import api_phase, get_api_tracker, patch_client_for_api_tracking

sys.path.append(str(Path(__file__).parent.parent.parent / "src"))

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.graph import create_graph
from onshape_robotics_toolkit.log import LOGGER, LogLevel
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import get_instances, get_mates_and_relations, get_parts, get_subassemblies
from onshape_robotics_toolkit.robot import get_robot


@dataclass
class PerformanceMetrics:
    """Performance metrics for a single phase"""

    phase_name: str
    duration: float
    success: bool
    api_calls: int
    timestamp: str
    metadata: dict[str, Any] | None = None


@dataclass
class AssemblyComplexity:
    """Automatically detected assembly complexity metrics"""

    part_count: int
    subassembly_count: int
    rigid_subassembly_count: int
    mate_count: int
    instance_count: int
    occurrence_count: int
    total_api_calls: int
    complexity_score: float  # Calculated composite score
    complexity_class: str  # Auto-classified: "simple", "medium", "complex", "large"
    robot_links: int = 0  # Number of links in robot graph
    robot_joints: int = 0  # Number of joints in robot graph


@dataclass
class AssemblyProfile:
    """Complete performance profile for an assembly"""

    assembly_name: str
    assembly_url: str
    complexity: AssemblyComplexity
    total_duration: float
    phases: list[PerformanceMetrics]
    api_call_summary: dict[str, Any]
    success: bool
    error_messages: list[str]
    timestamp: str


class WorkflowProfiler:
    """Comprehensive profiling of CAD-to-URDF workflows"""

    def __init__(self, output_dir: Path | None = None):
        self.output_dir = output_dir or Path("profiling_results")
        self.output_dir.mkdir(exist_ok=True)

        self.results: list[AssemblyProfile] = []
        self.current_assembly_errors: list[str] = []
        self.api_tracker = get_api_tracker()

        # Enable API call tracking
        self.original_client_methods = patch_client_for_api_tracking()

        # Suppress noisy logging during profiling
        LOGGER.set_stream_level(LogLevel.WARNING)

    def profile_assembly_workflow(self, assembly_url: str, assembly_name: str = "test_assembly") -> AssemblyProfile:
        """Profile complete workflow for a single assembly"""
        print(f"\n[PROFILING] Profiling: {assembly_name}")
        print(f"   URL: {assembly_url}")

        self.current_assembly_errors = []
        start_time = time.perf_counter()

        # Clear API tracker for this assembly
        self.api_tracker.clear()

        phases = []
        success = True

        # Define workflow phases
        workflow_phases = [
            ("client_setup", self._profile_client_setup),
            ("document_parsing", lambda: self._profile_document_parsing(assembly_url)),
            ("assembly_fetch", self._profile_assembly_fetch),
            ("instance_parsing", self._profile_instance_parsing),
            ("subassembly_processing", self._profile_subassembly_processing),
            ("part_processing", self._profile_part_processing),
            ("mate_processing", self._profile_mate_processing),
            ("graph_construction", self._profile_graph_construction),
            ("robot_generation", self._profile_robot_generation),
            ("robot_save", self._profile_robot_save),
        ]

        # Execute each phase with profiling
        for phase_name, phase_func in workflow_phases:
            try:
                print(f"   [PROGRESS] {phase_name}...", end=" ")
                metrics = self._profile_phase(phase_name, phase_func)
                phases.append(metrics)

                if metrics.success:
                    print(f"[SUCCESS] {metrics.duration:.3f}s")
                else:
                    print(f"[FAILED] {metrics.duration:.3f}s (failed)")
                    success = False

            except Exception as e:
                print(f"[FAILED] Failed: {e}")
                self.current_assembly_errors.append(f"{phase_name}: {e!s}")
                success = False

                # Add failed metrics
                phases.append(
                    PerformanceMetrics(
                        phase_name=phase_name,
                        duration=0.0,
                        success=False,
                        api_calls=0,
                        timestamp=datetime.now().isoformat(),
                        metadata=None,
                    )
                )

        total_duration = time.perf_counter() - start_time

        # Calculate assembly complexity from collected data
        complexity = self._calculate_assembly_complexity(phases)

        # Get API call summary
        api_summary = self.api_tracker.get_summary()

        profile = AssemblyProfile(
            assembly_name=assembly_name,
            assembly_url=assembly_url,
            complexity=complexity,
            total_duration=total_duration,
            phases=phases,
            api_call_summary=asdict(api_summary),
            success=success,
            error_messages=self.current_assembly_errors,
            timestamp=datetime.now().isoformat(),
        )

        self.results.append(profile)
        return profile

    def _calculate_assembly_complexity(self, phases: list[PerformanceMetrics]) -> AssemblyComplexity:
        """Calculate assembly complexity from phase metadata"""
        part_count = 0
        subassembly_count = 0
        rigid_subassembly_count = 0
        mate_count = 0
        instance_count = 0
        occurrence_count = 0
        robot_links = 0
        robot_joints = 0

        # Extract metrics from successful phases
        for phase in phases:
            if phase.success and phase.metadata:
                if "final_part_count" in phase.metadata:
                    part_count = phase.metadata["final_part_count"]
                elif "part_count" in phase.metadata:
                    part_count = max(part_count, phase.metadata["part_count"])

                if "subassembly_count" in phase.metadata:
                    subassembly_count = phase.metadata["subassembly_count"]
                if "rigid_subassembly_count" in phase.metadata:
                    rigid_subassembly_count = phase.metadata["rigid_subassembly_count"]
                if "mate_count" in phase.metadata:
                    mate_count = phase.metadata["mate_count"]
                if "instance_count" in phase.metadata:
                    instance_count = phase.metadata["instance_count"]
                if "occurrence_count" in phase.metadata:
                    occurrence_count = phase.metadata["occurrence_count"]

                # Robot graph data (nodes = links, edges = joints)
                if "link_count" in phase.metadata:
                    robot_links = phase.metadata["link_count"]
                if "joint_count" in phase.metadata:
                    robot_joints = phase.metadata["joint_count"]

        # Get API call count
        api_summary = self.api_tracker.get_summary()
        total_api_calls = api_summary.total_calls

        # Calculate composite complexity score
        complexity_score = (
            part_count * 1.0
            + subassembly_count * 2.0
            + rigid_subassembly_count * 1.5
            + mate_count * 0.5
            + instance_count * 0.3
            + occurrence_count * 0.2
        )

        # Auto-classify complexity based on part count
        def classify_complexity(parts: int) -> str:
            if parts <= 10:
                return "simple"
            elif parts <= 50:
                return "medium"
            elif parts <= 200:
                return "complex"
            else:
                return "large"

        complexity_class = classify_complexity(part_count)

        return AssemblyComplexity(
            part_count=part_count,
            subassembly_count=subassembly_count,
            rigid_subassembly_count=rigid_subassembly_count,
            mate_count=mate_count,
            instance_count=instance_count,
            occurrence_count=occurrence_count,
            total_api_calls=total_api_calls,
            complexity_score=complexity_score,
            complexity_class=complexity_class,
            robot_links=robot_links,
            robot_joints=robot_joints,
        )

    def _profile_phase(self, name: str, func) -> PerformanceMetrics:
        """Profile individual phase timing and API calls"""
        start_time = time.perf_counter()
        start_api_count = len(self.api_tracker.calls)
        metadata = None

        try:
            with api_phase(name):
                metadata = func()
            success = True
        except Exception as e:
            success = False
            self.current_assembly_errors.append(f"{name}: {e!s}")

        duration = time.perf_counter() - start_time
        end_api_count = len(self.api_tracker.calls)
        api_calls = end_api_count - start_api_count

        return PerformanceMetrics(
            phase_name=name,
            duration=duration,
            success=success,
            api_calls=api_calls,
            timestamp=datetime.now().isoformat(),
            metadata=metadata,
        )

    # Phase-specific profiling methods
    def _profile_client_setup(self) -> dict[str, Any]:
        """Profile client initialization"""
        with api_phase("client_setup"):
            self.client = Client(env=".env")
        return {"phase": "client_setup"}

    def _profile_document_parsing(self, url: str) -> dict[str, Any]:
        """Profile document URL parsing"""
        with api_phase("document_parsing"):
            self.document = Document.from_url(url)
            self.client.set_base_url(self.document.base_url)
        return {
            "phase": "document_parsing",
            "did": self.document.did,
            "wid": self.document.wid,
            "eid": self.document.eid,
        }

    def _profile_assembly_fetch(self) -> dict[str, Any]:
        """Profile assembly data fetching"""
        with api_phase("assembly_fetch"):
            self.assembly = self.client.get_assembly(
                did=self.document.did,
                wtype=self.document.wtype,
                wid=self.document.wid,
                eid=self.document.eid,
                log_response=False,
                with_meta_data=True,
            )

        # Estimate part count from assembly
        part_count = len(getattr(self.assembly, "parts", []))

        return {"phase": "assembly_fetch", "part_count": part_count, "assembly_size_estimate": part_count}

    def _profile_instance_parsing(self) -> dict[str, Any]:
        """Profile instance parsing"""
        with api_phase("instance_parsing"):
            self.instances, self.occurrences, self.id_to_name_map = get_instances(assembly=self.assembly, max_depth=0)

        return {
            "phase": "instance_parsing",
            "instance_count": len(self.instances),
            "occurrence_count": len(self.occurrences),
        }

    def _profile_subassembly_processing(self) -> dict[str, Any]:
        """Profile subassembly processing"""
        with api_phase("subassembly_processing"):
            self.subassemblies, self.rigid_subassemblies = get_subassemblies(
                assembly=self.assembly, client=self.client, instances=self.instances
            )

        return {
            "phase": "subassembly_processing",
            "subassembly_count": len(self.subassemblies),
            "rigid_subassembly_count": len(self.rigid_subassemblies),
        }

    def _profile_part_processing(self) -> dict[str, Any]:
        """Profile part processing"""
        with api_phase("part_processing"):
            self.parts = get_parts(
                assembly=self.assembly,
                rigid_subassemblies=self.rigid_subassemblies,
                client=self.client,
                instances=self.instances,
            )

        return {"phase": "part_processing", "part_count": len(self.parts), "final_part_count": len(self.parts)}

    def _profile_mate_processing(self) -> dict[str, Any]:
        """Profile mate and relation processing"""
        with api_phase("mate_processing"):
            self.mates, self.relations = get_mates_and_relations(
                assembly=self.assembly,
                subassemblies=self.subassemblies,
                rigid_subassemblies=self.rigid_subassemblies,
                id_to_name_map=self.id_to_name_map,
                parts=self.parts,
            )

        return {"phase": "mate_processing", "mate_count": len(self.mates), "relation_count": len(self.relations)}

    def _profile_graph_construction(self) -> dict[str, Any]:
        """Profile graph construction"""
        with api_phase("graph_construction"):
            self.graph, self.root_node = create_graph(
                occurrences=self.occurrences,
                instances=self.instances,
                parts=self.parts,
                mates=self.mates,
                use_user_defined_root=False,
            )

        return {
            "phase": "graph_construction",
            "node_count": len(self.graph.nodes),
            "edge_count": len(self.graph.edges),
            "root_node": self.root_node,
        }

    def _profile_robot_generation(self) -> dict[str, Any]:
        """Profile robot generation"""
        with api_phase("robot_generation"):
            self.robot = get_robot(
                assembly=self.assembly,
                graph=self.graph,
                root_node=self.root_node,
                parts=self.parts,
                mates=self.mates,
                relations=self.relations,
                client=self.client,
                robot_name="profiling_robot",
            )

        return {
            "phase": "robot_generation",
            "link_count": len(self.robot.graph.nodes),
            "joint_count": len(self.robot.graph.edges),
        }

    def _profile_robot_save(self) -> dict[str, Any]:
        """Profile robot save"""
        with api_phase("robot_save"):
            self.robot.save(file_path="profiling_robot.urdf")
        return {"phase": "robot_save"}

    def _profile_urdf_generation(self) -> dict[str, Any]:
        """Profile URDF generation"""
        urdf_content = self.robot.to_urdf()

        return {
            "phase": "urdf_generation",
            "urdf_length": len(urdf_content),
            "urdf_lines": len(urdf_content.splitlines()),
        }

    def get_timing_summary(self) -> dict[str, Any]:
        """Get simple timing summary of all profiles"""
        if not self.results:
            return {"error": "No profiling results available"}

        # Aggregate timing data
        phase_stats = {}
        for profile in self.results:
            for phase in profile.phases:
                if phase.phase_name not in phase_stats:
                    phase_stats[phase.phase_name] = {"times": [], "api_calls": [], "success_count": 0}

                phase_stats[phase.phase_name]["times"].append(phase.duration)
                phase_stats[phase.phase_name]["api_calls"].append(phase.api_calls)
                if phase.success:
                    phase_stats[phase.phase_name]["success_count"] += 1

        # Calculate summary statistics
        summary = {}
        for phase_name, stats in phase_stats.items():
            times = stats["times"]
            api_calls = stats["api_calls"]
            summary[phase_name] = {
                "avg_time": statistics.mean(times),
                "total_time": sum(times),
                "avg_api_calls": statistics.mean(api_calls),
                "total_api_calls": sum(api_calls),
                "success_rate": stats["success_count"] / len(times),
                "sample_count": len(times),
            }

        return {
            "total_profiles": len(self.results),
            "successful_profiles": sum(1 for p in self.results if p.success),
            "phase_stats": summary,
        }

    def save_results(self, filename: str | None = None) -> Path:
        """Save profiling results to JSON file"""
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"workflow_profile_{timestamp}.json"

        output_file = self.output_dir / filename

        # Convert results to JSON-serializable format
        results_data = {
            "metadata": {
                "timestamp": datetime.now().isoformat(),
                "total_profiles": len(self.results),
                "successful_profiles": sum(1 for p in self.results if p.success),
            },
            "profiles": [asdict(profile) for profile in self.results],
            "timing_summary": self.get_timing_summary(),
        }

        with open(output_file, "w") as f:
            json.dump(results_data, f, indent=2)

        print(f"\n[SAVED] Results saved to: {output_file}")
        return output_file

    def print_summary(self):
        """Print a summary of profiling results"""
        if not self.results:
            print("No profiling results to display")
            return

        summary = self.get_timing_summary()

        print(f"\n{'=' * 60}")
        print("WORKFLOW PROFILING SUMMARY")
        print(f"{'=' * 60}")
        print(f"Total assemblies profiled: {summary['total_profiles']}")
        print(f"Successful profiles: {summary['successful_profiles']}")
        print(f"Success rate: {summary['successful_profiles'] / summary['total_profiles'] * 100:.1f}%")

        phase_stats = summary.get("phase_stats", {})
        if phase_stats:
            print(f"\n{'PHASE PERFORMANCE':^60}")
            print(f"{'-' * 60}")
            print(f"{'Phase':<25} {'Avg Time (s)':<12} {'Success Rate':<12} {'API Calls'}")
            print(f"{'-' * 60}")

            for phase_name, stats in phase_stats.items():
                avg_time = stats["avg_time"]
                success_rate = stats["success_rate"]
                avg_api = stats["avg_api_calls"]

                print(f"{phase_name:<25} {avg_time:<12.3f} {success_rate:<12.1%} {avg_api:<8.1f}")

        print(f"\n{'TIMING DATA':^60}")
        print(f"{'-' * 60}")
        print("Review the JSON output file for detailed phase-by-phase analysis.")

        print(f"{'=' * 60}")


def main():
    """Main profiling execution"""
    print("[PROFILING] Onshape Robotics Toolkit - Workflow Profiler")
    print("=" * 60)

    profiler = WorkflowProfiler()

    # Test assemblies - replace with real URLs
    test_assemblies = [
        {
            "name": "quadruped",
            "url": "https://cad.onshape.com/documents/cf6b852d2c88d661ac2e17e8/w/c842455c29cc878dc48bdc68/e/b5e293d409dd0b88596181ef",
        },
        # Add more test assemblies here as needed
        # {
        #     "name": "simple_robot",
        #     "url": "https://cad.onshape.com/documents/.../simple"
        # }
    ]

    print(f"Will profile {len(test_assemblies)} assemblies")

    for assembly in test_assemblies:
        try:
            profiler.profile_assembly_workflow(assembly["url"], assembly["name"])
        except Exception as e:
            print(f"[FAILED] Failed to profile {assembly['name']}: {e}")

    # Print summary and save results
    profiler.print_summary()
    profiler.save_results()

    print("\n[SUCCESS] Profiling complete! Check the results to identify optimization targets.")


if __name__ == "__main__":
    main()
