# UBC Okanagan Marine Robotics AUV Software

## Overview

This repository contains the software for the UBC Okanagan Marine Robotics AUVs.

ROS 2 (Jazzy) is heavily utilized for inter-process communication, software configuration, and package management. We also utilize Devcontainers to ensure a consistent development environment across all machines.

Other software libraries used are dependent on specific packages. eg.
- `okmr_object_detection` uses PyTorch and ONNX
- `okmr_controls` uses Eigen 
- `okmr_mapping` uses PCL

### Package Structure
The codebase is organized into different ros2 packages, with each one containing code related to a 
specific area of the system. For example:
- `okmr_controls`: PID related code
- `okmr_navigation`: Navigation and movement coordination
- `okmr_automated_planner`: High level decision making state machines
- `stonefish_vendor`: A wrapper ensuring the simulator builds 

Check the specific README.md files inside each package for more details.


## Getting Started

We use **VS Code Devcontainers** to manage dependencies. You do not need to manually install ROS 2 or Stonefish on your host machine.

### Prerequisites
1.  **VS Code**: [Download here](https://code.visualstudio.com/)
2.  **Docker Desktop**: [Download here](https://www.docker.com/products/docker-desktop/)
3.  **VS Code Dev Containers Extension**: Install `ms-vscode-remote.remote-containers` from the VS Code Marketplace.

#### Windows Users (WSL 2) - Graphics Setup
To view the Stonefish simulator window, you must run an X Server on Windows:
1.  Download and install **[VcXsrv](https://sourceforge.net/projects/vcxsrv/)**.
2.  Launch **XLaunch** with these specific settings:
    * Display settings: **Multiple windows**
    * Display number: **0**
    * **Next >**
    * Start no client
    * **Next >**
    * Make sure **Disable access control** is checked
    * **Finish**

### Installation

1.  **Clone the Repository**:
    ```bash
    git clone --recursive https://github.com/UBCO-Marine-Robotics/okmr_auv.git
    cd okmr_auv
    ```

2.  **Open in Devcontainer**:
    * Open the folder in VS Code.
    * Press `F1` and select **"Dev Containers: Reopen in Container"**.
    * *Note: The first launch will take 5-10 minutes to build the Docker image.*

3.  **Build the Workspace**:
    Inside the VS Code integrated terminal:
    ```bash
    # Build all packages
    colcon build --symlink-install
    
    # Source the environment
    source install/setup.bash
    ```

### Running the Simulator
To verify the build, launch the Stonefish simulator.

**For WSL/Windows Users:**
You must export the display variable first (run in the integrated terminal):
```bash
export DISPLAY=$(grep nameserver /etc/resolv.conf | awk '{print $2}'):0
````

**Launch Command:**

```bash
ros2 launch okmr_stonefish sim.launch.py scenario_name:=simple.scn
```


## Controls Hierarchy

The overall system is designed in a layered manner, with L5 systems making high level decisions,
and every system below that being a step in the chain that makes the L5 systems request a reality.

Refresh / processing rate is the main factor that determines what layer a subsystem is in. The slower, the higher the layer.

This also strongly defines what programming language the code should be written in.

  - **L5 = Mission Plan** (e.g., root state machine in Automated Planner) ~30 second period
  - **L4 = Behavior Logic** (specific task state machines, motion planner) ~5 second period
  - **L3 = Action Coordinators and Perception** (navigator, mapper, object detection, system health) 5-20hz
  - **L2 = PID manager and Action Executors** 20-200hz
  - **L1 = PID controllers** ~200hz
  - **L0 = Hardware I/O** (motor/sensor interfaces) ~200hz


### System Diagram

![System Diagram](/diagrams/SystemDiagram.png)