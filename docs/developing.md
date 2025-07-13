# Developer Documentation

This document provides an overview of how the onshape-robotics-toolkit library works internally and how different components interact with each other.

## Architecture Overview

The library is organized into several key components:

- **Client**: Handles all communication with the Onshape API
- **Robot**: Manages the robot model creation and URDF export
- **Assembly**: Represents the CAD assembly structure
- **Utilities**: Helper functions for various operations

## Workflow Overview

### 1. Authentication and Connection

The library first establishes a secure connection with Onshape:

- Users provide API credentials (access key and secret key)
- The Client component handles authentication and maintains the session
- All subsequent API requests use this authenticated session

### 2. Document Access

Once authenticated:

- The library can access Onshape documents using either URLs or direct document IDs
- Document metadata is retrieved to verify access and get necessary identifiers
- Assembly information is cached to minimize API calls

### 3. Assembly Processing

The assembly processing workflow:

1. Retrieves the full assembly structure from Onshape
2. Parses the hierarchical relationship between components
3. Identifies joints, mate connectors, and other relevant features
4. Creates an internal representation of the robot structure

### 4. URDF Generation

The URDF export process:

1. Analyzes the assembly structure
2. Maps Onshape constraints to URDF joints
3. Exports geometry in specified formats
4. Generates a complete URDF file with proper kinematics

## Sequence Diagram

The following sequence diagram illustrates the main interactions between components:

<pre class="mermaid">
sequenceDiagram
    participant User as User
    participant Client as onshape-robotics-toolkit.client
    participant Onshape as Onshape Server
    participant Robot as onshape-robotics-toolkit.robot

    User->>Client: Initialize Client with API Keys
    Client->>Onshape: Authenticate with Access Key & Secret Key
    Onshape-->>Client: Return Authentication Token

    User->>Client: Request Document Information (e.g., document URL)
    Client->>Onshape: Send GET request for Document Details
    Onshape-->>Client: Return Document Metadata (JSON)
    Client-->>User: Deliver Document Metadata

    User->>Client: Request CAD Assembly
    Client->>Onshape: Send GET request for Assembly Details
    Onshape-->>Client: Return Assembly Data (JSON)
    Client-->>User: Deliver Assembly Data

    User->>Robot: Initiate URDF Export Workflow
    Robot-->>Onshape: Parse Assembly Data for URDF
    Robot-->>User: Deliver Robot model (URDF)

    User->>User: Use URDF for Simulation or Control
</pre>

## Key Classes and Their Roles

### Client Class

- Manages API authentication
- Handles all HTTP requests to Onshape
- Implements rate limiting and error handling
- Caches responses when appropriate

### Robot Class

- Represents the complete robot model
- Manages the conversion from CAD assembly to URDF
- Handles coordinate transformations
- Provides visualization utilities

### Assembly Class

- Represents the CAD assembly structure
- Maintains parent-child relationships
- Tracks mate connections and constraints
- Manages geometric transformations
