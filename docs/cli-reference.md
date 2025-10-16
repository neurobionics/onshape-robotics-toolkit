# CLI Reference

The Onshape Robotics Toolkit provides a comprehensive command-line interface (CLI) through the `ort` command. This reference documents all available commands and their options.

## Installation

After installing the package, the `ort` command should be available globally:

```bash
pip install onshape-robotics-toolkit
ort --help
```

You can also run it as a Python module:

```bash
python -m onshape_robotics_toolkit --help
```

## Global Options

These options work with any command:

| Option | Description |
|--------|-------------|
| `--help` | Show help message and exit |
| `--version` | Show toolkit version |

## Environment Variables

The CLI respects the following environment variables:

| Variable | Description | Default |
|----------|-------------|---------|
| `ORT_CONFIG` | Path to configuration file | `ORT.yaml` |
| `ORT_ENV` | Path to .env file | `.env` |
| `ORT_MAX_DEPTH` | Default max depth | `1` |
| `ORT_FORMAT` | Default export format | `urdf` |
| `ORT_MESH_DIR` | Default mesh directory | `meshes` |

## Commands

### `ort export`

Export an Onshape assembly to URDF or MJCF format.

**Usage:**
```bash
ort export <URL> <OUTPUT_PATH> [OPTIONS]
```

**Arguments:**
- `URL` - Onshape document URL (required)
- `OUTPUT_PATH` - Output file path, e.g., `robot.urdf` or `robot.xml` (required)

**Options:**

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `--format`, `-f` | choice | `urdf` | Export format: `urdf` or `mjcf` |
| `--name`, `-n` | text | (from filename) | Robot name |
| `--max-depth`, `-d` | integer | `1` | Maximum assembly depth (0=all rigid, higher=more flexible) |
| `--mesh-dir`, `-m` | path | `meshes` | Directory for mesh files |
| `--download-assets` / `--no-download-assets` | flag | `True` | Download STL assets |
| `--fetch-mass` / `--no-fetch-mass` | flag | `True` | Fetch mass properties from Onshape |
| `--use-user-root` / `--no-use-user-root` | flag | `True` | Use user-defined root part |
| `--position`, `-p` | float float float | - | Robot position (x y z), MJCF only |
| `--ground-plane` / `--no-ground-plane` | flag | `False` | Add ground plane (MJCF only) |
| `--config`, `-c` | path | - | Path to ORT.yaml configuration file |
| `--env`, `-e` | path | `.env` | Path to .env file |
| `--verbose`, `-v` | flag | `False` | Enable verbose (DEBUG) logging |
| `--quiet`, `-q` | flag | `False` | Quiet mode (file logging only) |

**Examples:**

```bash
# Basic URDF export
ort export "https://cad.onshape.com/documents/..." robot.urdf

# MJCF with options
ort export "https://cad.onshape.com/documents/..." robot.xml \
  --format mjcf \
  --max-depth 2 \
  --ground-plane \
  --position 0 0 0.5

# Using config file
ort export "https://cad.onshape.com/documents/..." robot.urdf \
  --config my-config.yaml

# Verbose mode
ort export "https://cad.onshape.com/documents/..." robot.urdf \
  --verbose
```

---

### `ort var`

Variable manipulation commands for updating Onshape variable studios.

#### `ort var list`

List all variables in an Onshape document.

**Usage:**
```bash
ort var list <URL> [OPTIONS]
```

**Options:**

| Option | Description |
|--------|-------------|
| `--json` | Output as JSON instead of table |
| `--env`, `-e` | Path to .env file |
| `--verbose`, `-v` | Enable verbose logging |
| `--quiet`, `-q` | Quiet mode |

**Example:**

```bash
ort var list "https://cad.onshape.com/documents/..."
ort var list "https://cad.onshape.com/documents/..." --json
```

#### `ort var get`

Get a specific variable value.

**Usage:**
```bash
ort var get <URL> --name <VARIABLE_NAME> [OPTIONS]
```

**Options:**

| Option | Required | Description |
|--------|----------|-------------|
| `--name`, `-n` | Yes | Variable name |
| `--env`, `-e` | No | Path to .env file |
| `--verbose`, `-v` | No | Enable verbose logging |
| `--quiet`, `-q` | No | Quiet mode (only output value) |

**Example:**

```bash
ort var get "https://cad.onshape.com/documents/..." --name wheelDiameter
```

#### `ort var set`

Set a variable value in an Onshape document.

**Usage:**
```bash
ort var set <URL> --name <NAME> --expression <EXPRESSION> [OPTIONS]
```

**Options:**

| Option | Required | Description |
|--------|----------|-------------|
| `--name`, `-n` | Yes | Variable name |
| `--expression`, `-e`, `--value` | Yes | Variable expression (e.g., "180 mm", "20 deg") |
| `--env` | No | Path to .env file |
| `--verbose`, `-v` | No | Enable verbose logging |
| `--quiet`, `-q` | No | Quiet mode |

**Example:**

```bash
ort var set "https://cad.onshape.com/documents/..." \
  --name wheelDiameter \
  --expression "180 mm"
```

#### `ort var export`

Update multiple variables and export in one command.

**Usage:**
```bash
ort var export <URL> <OUTPUT> [OPTIONS]
```

**Options:**

| Option | Description |
|--------|-------------|
| `--set`, `-s` | Set variable (format: `name=expression`), can be used multiple times |
| `--format`, `-f` | Export format: `urdf` or `mjcf` |
| `--max-depth`, `-d` | Maximum assembly depth |
| `--env` | Path to .env file |
| `--verbose`, `-v` | Enable verbose logging |
| `--quiet`, `-q` | Quiet mode |

**Example:**

```bash
ort var export "https://cad.onshape.com/documents/..." robot.urdf \
  --set wheelDiameter="180 mm" \
  --set forkAngle="20 deg" \
  --set plateThickness="5 mm"
```

---

### `ort assembly`

Assembly inspection commands.

#### `ort assembly info`

Show detailed information about an Onshape assembly.

**Usage:**
```bash
ort assembly info <URL> [OPTIONS]
```

**Options:**

| Option | Description |
|--------|-------------|
| `--max-depth`, `-d` | Maximum assembly depth |
| `--env`, `-e` | Path to .env file |
| `--verbose`, `-v` | Enable verbose logging |
| `--quiet`, `-q` | Quiet mode |

**Output includes:**
- Document information
- Part and instance counts
- Subassembly breakdown (flexible vs rigid)
- Mate statistics by type
- Kinematic graph structure

**Example:**

```bash
ort assembly info "https://cad.onshape.com/documents/..."
```

#### `ort assembly elements`

List all elements in an Onshape document.

**Usage:**
```bash
ort assembly elements <URL> [OPTIONS]
```

**Options:**

| Option | Description |
|--------|-------------|
| `--env`, `-e` | Path to .env file |
| `--verbose`, `-v` | Enable verbose logging |
| `--quiet`, `-q` | Quiet mode |

**Example:**

```bash
ort assembly elements "https://cad.onshape.com/documents/..."
```

---

### `ort graph`

Kinematic graph utilities.

#### `ort graph show`

Display the kinematic graph structure as a tree.

**Usage:**
```bash
ort graph show <URL> [OPTIONS]
```

**Options:**

| Option | Description |
|--------|-------------|
| `--max-depth`, `-d` | Maximum assembly depth |
| `--use-user-root` / `--no-use-user-root` | Use user-defined root part |
| `--env`, `-e` | Path to .env file |
| `--verbose`, `-v` | Enable verbose logging |
| `--quiet`, `-q` | Quiet mode |

**Example:**

```bash
ort graph show "https://cad.onshape.com/documents/..."
```

#### `ort graph export`

Export kinematic graph visualization to a file.

**Usage:**
```bash
ort graph export <URL> <OUTPUT> [OPTIONS]
```

**Arguments:**
- `OUTPUT` - Output file path (`.png`, `.svg`, `.pdf`, or `.dot`)

**Options:**

| Option | Description |
|--------|-------------|
| `--max-depth`, `-d` | Maximum assembly depth |
| `--use-user-root` / `--no-use-user-root` | Use user-defined root part |
| `--env`, `-e` | Path to .env file |
| `--verbose`, `-v` | Enable verbose logging |
| `--quiet`, `-q` | Quiet mode |

**Supported formats:**
- `.png` - PNG image (requires matplotlib)
- `.svg` - SVG vector image (requires matplotlib)
- `.pdf` - PDF document (requires matplotlib)
- `.dot` - GraphViz DOT format

**Example:**

```bash
ort graph export "https://cad.onshape.com/documents/..." graph.png
ort graph export "https://cad.onshape.com/documents/..." graph.dot
```

---

### `ort robot`

Robot model commands.

#### `ort robot info`

Show detailed information about the robot model.

**Usage:**
```bash
ort robot info <URL> [OPTIONS]
```

**Options:**

| Option | Description |
|--------|-------------|
| `--max-depth`, `-d` | Maximum assembly depth |
| `--name`, `-n` | Robot name |
| `--env`, `-e` | Path to .env file |
| `--verbose`, `-v` | Enable verbose logging |
| `--quiet`, `-q` | Quiet mode |

**Output includes:**
- Robot name, link count, joint count
- Root link information
- Joint type breakdown
- Link details (mass properties)
- Asset information

**Example:**

```bash
ort robot info "https://cad.onshape.com/documents/..."
```

#### `ort robot show-tree`

Display the robot structure as a tree with joints.

**Usage:**
```bash
ort robot show-tree <URL> [OPTIONS]
```

**Options:**

| Option | Description |
|--------|-------------|
| `--max-depth`, `-d` | Maximum assembly depth |
| `--name`, `-n` | Robot name |
| `--env`, `-e` | Path to .env file |
| `--verbose`, `-v` | Enable verbose logging |
| `--quiet`, `-q` | Quiet mode |

**Example:**

```bash
ort robot show-tree "https://cad.onshape.com/documents/..."
```

---

### `ort config`

Configuration management commands.

#### `ort config init`

Initialize a new configuration file interactively.

**Usage:**
```bash
ort config init [OPTIONS]
```

**Options:**

| Option | Description |
|--------|-------------|
| `--output`, `-o` | Output file path (default: `ORT.yaml`) |
| `--force`, `-f` | Overwrite existing file |

**Example:**

```bash
ort config init
ort config init --output my-config.yaml
```

#### `ort config validate`

Validate an existing configuration file.

**Usage:**
```bash
ort config validate [CONFIG_PATH] [OPTIONS]
```

**Arguments:**
- `CONFIG_PATH` - Path to configuration file (default: `ORT.yaml`)

**Options:**

| Option | Description |
|--------|-------------|
| `--verbose`, `-v` | Enable verbose output |

**Checks performed:**
- Configuration schema validation
- .env file existence and contents
- File path validation
- Configuration completeness

**Example:**

```bash
ort config validate
ort config validate my-config.yaml
```

#### `ort config show`

Display configuration file with syntax highlighting.

**Usage:**
```bash
ort config show [CONFIG_PATH] [OPTIONS]
```

**Arguments:**
- `CONFIG_PATH` - Path to configuration file (default: `ORT.yaml`)

**Options:**

| Option | Description |
|--------|-------------|
| `--no-color` | Disable syntax highlighting |

**Example:**

```bash
ort config show
ort config show my-config.yaml --no-color
```

#### `ort config edit`

Open configuration file in your default editor.

**Usage:**
```bash
ort config edit [CONFIG_PATH] [OPTIONS]
```

**Arguments:**
- `CONFIG_PATH` - Path to configuration file (default: `ORT.yaml`)

**Options:**

| Option | Description |
|--------|-------------|
| `--editor`, `-e` | Editor command (default: `$EDITOR` or `notepad`) |

**Example:**

```bash
ort config edit
ort config edit my-config.yaml --editor vim
```

---

### `ort version`

Show toolkit version information.

**Usage:**
```bash
ort version
```

**Example:**

```bash
$ ort version
Onshape Robotics Toolkit version 0.4.0
```

---

## Configuration File Format

The `ORT.yaml` configuration file uses the following structure:

```yaml
logging:
  mode: default
  console_level: INFO
  file_level: DEBUG
  file_path: ORT.log

client:
  env: .env
  base_url: https://cad.onshape.com

cad:
  max_depth: 1

kinematics:
  use_user_defined_root: true

robot:
  name: my_robot
  type: urdf
  fetch_mass_properties: true

export:
  download_assets: true
  mesh_dir: meshes

names:
  parts: {}
  mates: {}
```

### Configuration Precedence

Configuration values are resolved in the following order (highest to lowest priority):

1. Command-line arguments
2. Environment variables
3. Configuration file (`ORT.yaml`)
4. Default values

**Example:**

```bash
# CLI argument takes precedence
ort export <URL> robot.urdf --max-depth 2

# Environment variable
export ORT_MAX_DEPTH=2
ort export <URL> robot.urdf

# Config file value
# (set max_depth: 2 in ORT.yaml)
ort export <URL> robot.urdf --config ORT.yaml

# Default value (max_depth=1)
ort export <URL> robot.urdf
```

---

## Common Workflows

### Quick Export

Export an assembly with minimal configuration:

```bash
# Create .env with credentials
echo "ONSHAPE_ACCESS_KEY=..." >> .env
echo "ONSHAPE_SECRET_KEY=..." >> .env

# Export
ort export "https://cad.onshape.com/documents/..." robot.urdf
```

### Parametric Design Export

Update design parameters and export:

```bash
ort var export "https://cad.onshape.com/documents/..." robot.urdf \
  --set linkLength="500 mm" \
  --set jointAngle="45 deg" \
  --max-depth 2
```

### Batch Processing

Process multiple configurations:

```bash
for depth in 0 1 2; do
  ort export "https://cad.onshape.com/documents/..." \
    "robot_depth_${depth}.urdf" \
    --max-depth $depth
done
```

### Inspection Workflow

Understand assembly structure before export:

```bash
# View assembly info
ort assembly info "https://cad.onshape.com/documents/..."

# View kinematic graph
ort graph show "https://cad.onshape.com/documents/..."

# Check robot statistics
ort robot info "https://cad.onshape.com/documents/..."

# Export
ort export "https://cad.onshape.com/documents/..." robot.urdf
```

---

## Troubleshooting

### Authentication Errors

**Problem:** `Error: Missing environment variable: ONSHAPE_ACCESS_KEY`

**Solution:**
1. Create a `.env` file with your credentials
2. Or set environment variables manually:
   ```bash
   export ONSHAPE_ACCESS_KEY=your_key
   export ONSHAPE_SECRET_KEY=your_secret
   ```

### Invalid URL

**Problem:** `URL must start with 'https://cad.onshape.com/documents/'`

**Solution:** Ensure you're using the full Onshape document URL, not a shortened version.

### Empty Kinematic Graph

**Problem:** `Cannot create robot from empty kinematic graph`

**Solution:** Mark at least one part or subassembly as "fixed" in your Onshape assembly.

### Missing Dependencies

**Problem:** `Error: matplotlib is required for image export`

**Solution:** Install optional dependencies:
```bash
pip install matplotlib
```

---

## Getting Help

For additional help:

- View command help: `ort <command> --help`
- Full documentation: https://neurobionics.github.io/onshape-robotics-toolkit/
- Report issues: https://github.com/neurobionics/onshape-robotics-toolkit/issues
