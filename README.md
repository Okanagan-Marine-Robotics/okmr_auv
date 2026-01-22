# UBC Okanagan Marine Robotics AUV Software

## Overview

This repository contains the software for the UBC Okanagan Marine Robotics AUVs.

ROS 2 (Jazzy) is heavily utilized for inter-process communication, software configuration, and package management. We also utilize Devcontainers to ensure a consistent development environment across all machines.

Other software libraries used are dependent on specific packages. eg.

* `okmr_object_detection` uses PyTorch and ONNX
* `okmr_controls` uses Eigen
* `okmr_mapping` uses PCL

### Package Structure

The codebase is organized into different ros2 packages, with each one containing code related to a
specific area of the system. For example:

* `okmr_controls`: PID related code
* `okmr_navigation`: Navigation and movement coordination
* `okmr_automated_planner`: High level decision making state machines
* `stonefish_vendor`: A wrapper ensuring the simulator builds

Check the specific README.md files inside each package for more details.

## Getting Started

We use **VS Code Devcontainers** to manage dependencies. This ensures a consistent environment without manual installation of ROS 2 or Stonefish on your host.

### Prerequisites

1. **VS Code**: [Download here](https://code.visualstudio.com/)
2. **Docker Desktop**: [Download here](https://www.docker.com/products/docker-desktop/)
3. **VS Code Dev Containers Extension**: Install `ms-vscode-remote.remote-containers` from the VS Code Marketplace.

### GPU Acceleration Setup (For Stonefish)

#### Windows Users (WSL 2)

1. **Ensure you are on Windows 10 Build 19044+ or Windows 11.** * These versions include **WSLg**, which handles graphics automatically. 
2. **Install NVIDIA Drivers**: Ensure your host Windows machine has the latest NVIDIA drivers installed.

#### Linux Users

1. Install the **[NVIDIA Container Toolkit](https://www.google.com/search?q=https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)** on your host machine.
2. Restart Docker: `sudo systemctl restart docker`

### Installation

1. **Clone the Repository**:
```bash
git clone --recursive https://github.com/Okanagan-Marine-Robotics/okmr_auv.git
cd okmr_auv

```

2. **Open in Devcontainer**:
* Open the folder in VS Code:
```bash
code .

```

* Press `F1` and select **"Dev Containers: Reopen in Container"**.

*Note: The first launch will take a few minutes as it compiles the Stonefish simulator into the Docker image.*

3. **Build the Workspace**:
Inside the VS Code integrated terminal:
```bash
# Build all packages
colcon build --symlink-install

# Source the environment
source install/setup.bash

```


### Running the Simulator

To verify the build, launch the Stonefish simulator.

1. **Source the Workspace**:
```bash
source install/setup.bash

```

2. **Launch Command:**
```bash
ros2 launch okmr_stonefish sim.launch.py scenario_name:=simple.scn

```

## Controls Hierarchy

The overall system is designed in a layered manner, with L5 systems making high level decisions,
and every system below that being a step in the chain that makes the L5 systems request a reality.

Refresh / processing rate is the main factor that determines what layer a subsystem is in. The slower, the higher the layer.

This also strongly defines what programming language the code should be written in.

* **L5 = Mission Plan** (e.g., root state machine in Automated Planner) ~30 second period
* **L4 = Behavior Logic** (specific task state machines, motion planner) ~5 second period
* **L3 = Action Coordinators and Perception** (navigator, mapper, object detection, system health) 5-20hz
* **L2 = PID manager and Action Executors** 20-200hz
* **L1 = PID controllers** ~200hz
* **L0 = Hardware I/O** (motor/sensor interfaces) ~200hz

### System Diagram

![System Diagram](/diagrams/SystemDiagram.png)