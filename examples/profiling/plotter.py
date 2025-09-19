import json

import matplotlib.pyplot as plt
import numpy as np


def plot_results(data_json: str):
    """Plot the results of the profiling analysis"""
    with open(data_json) as f:
        data = json.load(f)

    # Plot the results
    plt.plot(data["x"], data["y"])
    plt.show()


def load_data(data_json: str):
    """Extract the data from the profiling analysis"""
    with open(data_json) as f:
        data = json.load(f)

    return data


def extract_profiles(data: dict):
    """Extract the profiles from the profiling analysis"""
    out = {}
    for profile in data["raw_profiles"]:
        out[profile["assembly_name"]] = profile

    return out


def extract_complexity(profiles: dict):
    """Extract the complexity from the profiling analysis"""
    out = {}
    for profile in profiles:
        out[profile] = profiles[profile]["complexity"]
    return out


def extract_api_calls_info(profiles: dict):
    """Extract the API calls info from the profiling analysis"""
    out = {}
    for profile in profiles:
        out[profile] = profiles[profile]["api_call_summary"]
    return out


if __name__ == "__main__":
    data = load_data("profiling_results/data.json")
    # plot_results(data)
    profiles = extract_profiles(data)
    complexity = extract_complexity(profiles)
    api_calls_info = extract_api_calls_info(profiles)

    # plot the complexity vs api calls with improved aesthetics
    x = []
    y = []
    names = []
    for profile in complexity:
        x.append(complexity[profile]["occurrence_count"])
        y.append(api_calls_info[profile]["total_calls"])
        names.append(profile)

    plt.figure(figsize=(10, 6))
    scatter = plt.scatter(x, y, c="tab:blue", s=80, alpha=0.8, edgecolors="k")

    # Annotate points with profile names
    for xi, yi, name in zip(x, y, names, strict=False):
        plt.annotate(name, (xi, yi), textcoords="offset points", xytext=(5, 5), ha="left", fontsize=9)

    plt.xlabel("Number of Parts or Rigid Subassemblies", fontsize=12)
    plt.ylabel("Number of API Calls", fontsize=12)
    plt.title("Complexity vs API Calls", fontsize=14, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.savefig("complexity_vs_api_calls.png", dpi=150)
    plt.close()

    # plot the average API calls per endpoint
    api_calls_per_endpoint = {}
    for profile in api_calls_info:
        for endpoint, calls in api_calls_info[profile]["calls_by_endpoint"].items():
            # sanitize endpoint name, only get name till '(' and remove Client.
            endpoint = endpoint.split("(")[0].strip()
            endpoint = endpoint.replace("Client.", "")
            if endpoint not in api_calls_per_endpoint:
                api_calls_per_endpoint[endpoint] = []
            api_calls_per_endpoint[endpoint].append(calls)
    avg_api_calls_per_endpoint = {endpoint: np.mean(calls) for endpoint, calls in api_calls_per_endpoint.items()}
    # Build the endpoints list as the union of all endpoints across all profiles
    all_endpoints = set()
    for profile in api_calls_info:
        all_endpoints.update(api_calls_info[profile]["calls_by_endpoint"].keys())
    # Sanitize endpoint names as before
    endpoints = sorted({ep.split("(")[0].strip().replace("Client.", "") for ep in all_endpoints})
    avg_calls = [avg_api_calls_per_endpoint.get(endpoint, 0) for endpoint in endpoints]
    plt.figure(figsize=(12, 6))
    plt.barh(endpoints, avg_calls, color="tab:blue", edgecolor="k")
    plt.xlabel("Average API Calls", fontsize=12)
    plt.title("Average API Calls per Endpoint", fontsize=14, fontweight="bold")
    plt.grid(axis="x", linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.savefig("avg_api_calls_per_endpoint.png", dpi=150)
    plt.close()

    # create a superimposed bar chart for API calls by endpoint and profile
    plt.figure(figsize=(12, 6))
    bar_width = 0.15
    indices = np.arange(len(endpoints))
    for i, profile in enumerate(profiles):
        # Use sanitized endpoint names for lookup
        calls = []
        for endpoint in endpoints:
            # Find the original endpoint key in the profile that matches the sanitized endpoint
            found = 0
            for ep_key in api_calls_info[profile]["calls_by_endpoint"]:
                ep_sanitized = ep_key.split("(")[0].strip().replace("Client.", "")
                if ep_sanitized == endpoint:
                    found = api_calls_info[profile]["calls_by_endpoint"][ep_key]
                    break
            calls.append(found)
        plt.bar(indices + i * bar_width, calls, width=bar_width, label=profile)
    plt.xlabel("API Endpoints", fontsize=12)
    plt.ylabel("Number of API Calls", fontsize=12)
    plt.title("API Calls by Endpoint and Complexity", fontsize=14, fontweight="bold")
    plt.xticks(indices + bar_width * (len(profiles) - 1) / 2, endpoints, rotation=45)
    plt.legend(title="Profiles")
    plt.tight_layout()
    plt.savefig("api_calls_by_endpoint_and_complexity.png", dpi=150)
    plt.close()

    # create a stacked bar chart for API calls by phase and profile
    plt.figure(figsize=(12, 6))
    phase_calls = {}
    for profile in profiles:
        for phase, calls in api_calls_info[profile]["calls_by_phase"].items():
            if phase not in phase_calls:
                phase_calls[phase] = []
            phase_calls[phase].append(calls)
    phases = sorted(phase_calls.keys())
    calls_by_phase = np.array([phase_calls[phase] for phase in phases]).T
    bar_width = 0.15
    indices = np.arange(len(phases))
    for i, profile in enumerate(profiles):
        plt.bar(indices + i * bar_width, calls_by_phase[i], width=bar_width, label=profile)
    plt.xlabel("Phases", fontsize=12)
    plt.ylabel("Number of API Calls", fontsize=12)
    plt.title("API Calls by Phase and Complexity", fontsize=14, fontweight="bold")
    plt.xticks(indices + bar_width * (len(profiles) - 1) / 2, phases, rotation=0)
    plt.legend(title="Profiles")
    plt.tight_layout()
    plt.savefig("api_calls_by_phase_and_complexity.png", dpi=150)
    plt.close()
    print("Plots saved successfully.")

    # plot average API calls per endpoint across all profiles
    avg_api_calls_per_endpoint = {endpoint: np.mean(calls) for endpoint, calls in api_calls_per_endpoint.items()}
    plt.figure(figsize=(12, 6))
    plt.barh(
        list(avg_api_calls_per_endpoint.keys()),
        list(avg_api_calls_per_endpoint.values()),
        color="tab:blue",
        edgecolor="k",
    )
    plt.xlabel("Average API Calls", fontsize=12)
    plt.title("Average API Calls per Endpoint Across All 3 Assemblies", fontsize=14, fontweight="bold")
    plt.grid(axis="x", linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.savefig("avg_api_calls_per_endpoint_across_profiles.png", dpi=150)
    plt.close()
