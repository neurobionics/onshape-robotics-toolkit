#!/usr/bin/env python3
"""
API Call Tracking for Onshape Robotics Toolkit

This module provides comprehensive tracking of API calls made during CAD-to-URDF workflows
to understand the relationship between assembly complexity and API usage.
"""

import contextlib
import json
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any


@dataclass
class ApiCall:
    """Individual API call record"""

    method: str
    endpoint: str
    timestamp: float
    duration: float
    success: bool
    status_code: int | None = None
    response_size: int | None = None
    phase: str | None = None  # Which workflow phase made this call
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class ApiCallSummary:
    """Summary of API calls for analysis"""

    total_calls: int
    successful_calls: int
    failed_calls: int
    total_duration: float
    average_duration: float
    calls_by_endpoint: dict[str, int]
    calls_by_phase: dict[str, int]
    response_size_total: int
    unique_endpoints: int


class ApiCallTracker:
    """Thread-safe API call tracking"""

    def __init__(self):
        self._calls: list[ApiCall] = []
        self._lock = threading.Lock()
        self._current_phase: str | None = None
        self._phase_stack: list[str] = []

    def set_current_phase(self, phase: str):
        """Set the current workflow phase for call attribution"""
        with self._lock:
            self._current_phase = phase

    def push_phase(self, phase: str):
        """Push a phase onto the stack (for nested operations)"""
        with self._lock:
            self._phase_stack.append(self._current_phase)
            self._current_phase = phase

    def pop_phase(self):
        """Pop the previous phase from the stack"""
        with self._lock:
            if self._phase_stack:
                self._current_phase = self._phase_stack.pop()
            else:
                self._current_phase = None

    def record_call(
        self,
        method: str,
        endpoint: str,
        duration: float,
        success: bool,
        status_code: int | None = None,
        response_size: int | None = None,
        **metadata,
    ):
        """Record an API call"""
        call = ApiCall(
            method=method,
            endpoint=self._normalize_endpoint(endpoint),
            timestamp=time.time(),
            duration=duration,
            success=success,
            status_code=status_code,
            response_size=response_size,
            phase=self._current_phase,
            metadata=metadata,
        )
        with self._lock:
            self._calls.append(call)

    def _normalize_endpoint(self, endpoint: str) -> str:
        """Normalize endpoint URLs for grouping"""
        # Remove document/workspace/element IDs to group similar calls
        import re

        # Replace UUIDs with placeholders
        uuid_pattern = r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}"
        endpoint = re.sub(uuid_pattern, "{id}", endpoint)

        # Replace other common ID patterns
        endpoint = re.sub(r"/[0-9a-f]{24}", "/{id}", endpoint)  # 24-char hex IDs
        endpoint = re.sub(r"/[0-9a-f]{32}", "/{id}", endpoint)  # 32-char hex IDs

        return endpoint

    @property
    def calls(self) -> list[ApiCall]:
        """Get all recorded calls (property for compatibility)"""
        with self._lock:
            return self._calls.copy()

    def get_calls(self) -> list[ApiCall]:
        """Get all recorded calls"""
        with self._lock:
            return self._calls.copy()

    def get_summary(self) -> ApiCallSummary:
        """Get summary statistics of API calls"""
        with self._lock:
            calls = self._calls.copy()

        if not calls:
            return ApiCallSummary(
                total_calls=0,
                successful_calls=0,
                failed_calls=0,
                total_duration=0.0,
                average_duration=0.0,
                calls_by_endpoint={},
                calls_by_phase={},
                response_size_total=0,
                unique_endpoints=0,
            )

        successful_calls = sum(1 for c in calls if c.success)
        failed_calls = len(calls) - successful_calls
        total_duration = sum(c.duration for c in calls)

        calls_by_endpoint = defaultdict(int)
        calls_by_phase = defaultdict(int)
        response_size_total = 0

        for call in calls:
            calls_by_endpoint[call.endpoint] += 1
            if call.phase:
                calls_by_phase[call.phase] += 1
            if call.response_size:
                response_size_total += call.response_size

        return ApiCallSummary(
            total_calls=len(calls),
            successful_calls=successful_calls,
            failed_calls=failed_calls,
            total_duration=total_duration,
            average_duration=total_duration / len(calls) if calls else 0,
            calls_by_endpoint=dict(calls_by_endpoint),
            calls_by_phase=dict(calls_by_phase),
            response_size_total=response_size_total,
            unique_endpoints=len(calls_by_endpoint),
        )

    def clear(self):
        """Clear all recorded calls"""
        with self._lock:
            self._calls.clear()
            self._current_phase = None
            self._phase_stack.clear()

    def save_detailed_log(self, filename: str):
        """Save detailed call log to JSON file"""
        with self._lock:
            calls = self._calls.copy()

        log_data = {
            "metadata": {
                "timestamp": datetime.now().isoformat(),
                "total_calls": len(calls),
                "tracking_session": "api_call_analysis",
            },
            "calls": [
                {
                    "method": call.method,
                    "endpoint": call.endpoint,
                    "timestamp": call.timestamp,
                    "duration": call.duration,
                    "success": call.success,
                    "status_code": call.status_code,
                    "response_size": call.response_size,
                    "phase": call.phase,
                    "metadata": call.metadata,
                }
                for call in calls
            ],
            "summary": self.get_summary().__dict__,
        }

        with open(filename, "w") as f:
            json.dump(log_data, f, indent=2, default=str)


# Global tracker instance
_api_tracker = ApiCallTracker()


def get_api_tracker() -> ApiCallTracker:
    """Get the global API call tracker"""
    return _api_tracker


def track_api_call(func):
    """Decorator to track API calls"""
    import functools

    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        # Extract method and endpoint info
        method = getattr(func, "__name__", "unknown")
        endpoint = kwargs.get("endpoint", f"{func.__module__}.{func.__name__}")

        start_time = time.perf_counter()
        success = False
        status_code = None
        response_size = None

        try:
            result = func(*args, **kwargs)
            success = True

            # Try to extract response info
            if hasattr(result, "__len__"):
                with contextlib.suppress(Exception):
                    response_size = len(str(result))

            return result

        finally:
            duration = time.perf_counter() - start_time
            _api_tracker.record_call(
                method=method,
                endpoint=endpoint,
                duration=duration,
                success=success,
                status_code=status_code,
                response_size=response_size,
            )

    return wrapper


class ApiCallContext:
    """Context manager for tracking API calls in specific phases"""

    def __init__(self, phase: str):
        self.phase = phase

    def __enter__(self):
        _api_tracker.push_phase(self.phase)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        _api_tracker.pop_phase()


def api_phase(phase: str):
    """Context manager for API call phase tracking"""
    return ApiCallContext(phase)


# Monkey-patch the Client class to track API calls
def patch_client_for_api_tracking():
    """Patch the Client class to automatically track API calls"""
    try:
        import sys
        from pathlib import Path

        # Add src to path
        sys.path.append(str(Path(__file__).parent.parent.parent / "src"))

        from onshape_robotics_toolkit.connect import Client

        # Store original methods
        original_methods = {}

        # Methods to track (based on actual usage in codebase)
        methods_to_track = [
            "get_assembly",
            "get_mass_property",
            "get_assembly_mass_properties",
            "get_elements",
            "get_document_metadata",
            "get_variables",
            "set_variables",
            "get_root_assembly",
            "get_assembly_name",
            "download_part_stl",
            "download_assembly_stl",
            "set_base_url",
        ]

        for method_name in methods_to_track:
            if hasattr(Client, method_name):
                original_method = getattr(Client, method_name)
                original_methods[method_name] = original_method

                # Create tracked version
                def make_tracked_method(original_func, name):
                    @functools.wraps(original_func)
                    def tracked_method(self, *args, **kwargs):
                        start_time = time.perf_counter()
                        success = False
                        status_code = None
                        response_size = None

                        try:
                            result = original_func(self, *args, **kwargs)
                            success = True
                            # Estimate response size

                            if hasattr(result, "__dict__"):
                                response_size = len(str(result.__dict__))
                            else:
                                response_size = len(str(result))

                        except Exception as e:
                            status_code = getattr(e, "status_code", None)
                            raise
                        else:
                            return result

                        finally:
                            duration = time.perf_counter() - start_time
                            # Extract meaningful endpoint info
                            endpoint_info = f"Client.{name}"
                            if args:
                                # Add first argument (usually document/element ID) for better tracking
                                endpoint_info += f"({args[0] if args else ''})"

                            _api_tracker.record_call(
                                method="HTTP",
                                endpoint=endpoint_info,
                                duration=duration,
                                success=success,
                                status_code=status_code,
                                response_size=response_size,
                                args_count=len(args),
                                kwargs_count=len(kwargs),
                            )

                    return tracked_method

                # Monkey patch the method
                import functools

                setattr(Client, method_name, make_tracked_method(original_method, method_name))

        print("[SUCCESS] API call tracking enabled for Client methods")

    except Exception as e:
        print(f"[WARNING]  Could not enable API tracking: {e}")
        return {}
    else:
        return original_methods


if __name__ == "__main__":
    # Test the API tracker
    tracker = get_api_tracker()

    # Simulate some API calls
    with api_phase("test_phase"):
        tracker.record_call("GET", "/api/documents/{id}", 0.5, True, 200, 1024)
        tracker.record_call("GET", "/api/assemblies/{id}", 1.2, True, 200, 2048)
        tracker.record_call("GET", "/api/parts/{id}/mass", 0.8, False, 500)

    # Print summary
    summary = tracker.get_summary()
    print("API Call Summary:")
    print(f"  Total calls: {summary.total_calls}")
    print(f"  Success rate: {summary.successful_calls}/{summary.total_calls}")
    print(f"  Average duration: {summary.average_duration:.3f}s")
    print(f"  Calls by phase: {summary.calls_by_phase}")
    print(f"  Unique endpoints: {summary.unique_endpoints}")
