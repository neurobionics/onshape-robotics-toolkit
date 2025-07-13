<!-- [![Release](https://img.shields.io/github/v/release/neurobionics/onshape-robotics-toolkit)](https://img.shields.io/github/v/release/neurobionics/onshape-robotics-toolkit)
[![Build status](https://img.shields.io/github/actions/workflow/status/neurobionics/onshape-robotics-toolkit/main.yml?branch=main)](https://github.com/neurobionics/onshape-robotics-toolkit/actions/workflows/main.yml?query=branch%3Amain)
[![Commit activity](https://img.shields.io/github/commit-activity/m/neurobionics/onshape-robotics-toolkit)](https://img.shields.io/github/commit-activity/m/neurobionics/onshape-robotics-toolkit)
[![License](https://img.shields.io/github/license/neurobionics/onshape-robotics-toolkit)](https://img.shields.io/github/license/neurobionics/onshape-robotics-toolkit) -->

# onshape-robotics-toolkit

`onshape-robotics-toolkit` is a feature-rich Python library that significantly extends the capabilities of Onshape's web-based CAD platform. The library provides a comprehensive API for automating robot design tasks, including solid model manipulation, robot assembly management, graph-based visualizations, and exporting CAD assemblies to URDF files for simulation and control. Intended as a resource for the robotics community, this library leverages Onshape's REST API to facilitate advanced workflows that bridge CAD design and robotics applications.

<img src="tutorials/export/export-header.gif" alt="Header" style="width: 100%;">

&nbsp;
This library was inspired by <a href="https://github.com/Rhoban/onshape-to-robot" target="_blank">onshape-to-robot</a>, a tool renowned for its streamlined approach to URDF generation. While onshape-to-robot library focuses on predefined workflows and design-time considerations, the `onshape-robotics-toolkit` library offers greater flexibility. It provides access to nearly all of Onshape's REST API calls, enabling headless manipulation, detailed analysis, and seamless export of CAD assemblies. Users can programmatically edit variable studios, generate graph-based visualizations, and export URDF files tailored to their specific needs—all without being restricted by rigid workflows or naming conventions. By removing these constraints, the `onshape-robotics-toolkit` library empowers the robotics and CAD communities to create custom solutions for algorithmic design, optimization, and automation.

&nbsp;

# Key Features of `onshape-robotics-toolkit`

The `onshape-robotics-toolkit` library is designed for users seeking a scalable, versatile API that empowers innovative robot design and control workflows. By integrating Onshape into algorithmic processes such as design optimization and automation, it unlocks the full potential of Onshape's cloud-based CAD system, fostering creativity and efficiency in robotics and beyond.

&nbsp;

| Feature                              | `onshape-robotics-toolkit`           | `onshape-to-robot`                      |
| ------------------------------------ | ------------------------------------ | --------------------------------------- |
| **Workflow Flexibility**             | ✅ Open-ended and customizable       | ❌ Predefined and rigid                 |
| **Design-Time Considerations**       | ✅ None                              | ❌ Requires specific naming conventions |
| **Custom URDF Workflow**             | ✅ Supports any assembly             | ❌ Limited by design rules              |
| **Variable Studio Editing**          | ✅ Yes                               | ❌ No                                   |
| **Ease of Setup**                    | ❌ Moderate (requires python coding) | ✅ Easy (no coding required)            |
| **Headless Integration**             | ✅ Yes (e.g., optimization)          | ❌ No out-of-the-box support            |
| **Access to Full Onshape API**       | ✅ Yes                               | ❌ Limited                              |
| **Graph Visualization and Analysis** | ✅ Supports graph generation         | ❌ Not supported                        |

## Prerequisites

Before you begin, ensure you have the following:

- <a href="https://www.python.org/downloads/release/python-3100/" target="_blank">Python 3.10</a> or higher installed on your machine.
- <a href="https://www.onshape.com/en/" target="_blank">An Onshape account</a> if you don't already have one.
- <a href="https://onshape-public.github.io/docs/auth/apikeys/" target="_blank">Onshape API keys (access key and secret key)</a>

## Installation

You can install `onshape-robotics-toolkit` using `pip`, which is the easiest way to install it and is the recommended method for most users.

```sh
pip install onshape-robotics-toolkit
```

If you want to install from source, you'll need to install [`uv`](https://docs.astral.sh/uv/getting-started/installation/) and [`git`](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) first. Then, you can clone the repository and install the package.

```sh
git clone https://github.com/neurobionics/onshape-robotics-toolkit.git
cd onshape-robotics-toolkit
uv sync
```

## Contributing

If you're interested in contributing to the project, please read the [contributing guidelines](contributing.md) to get started. All contributions are welcome!

## License

This project is licensed under the Apache 2.0 License. For more information, please refer to the [license](#) file.

## References

- [Onshape API Documentation](https://onshape-public.github.io/docs/)
- [Onshape API Glassworks Explorer](https://cad.onshape.com/glassworks/explorer/#/)
- [Onshape to Robot URDF Exporter](https://github.com/Rhoban/onshape-to-robot)
