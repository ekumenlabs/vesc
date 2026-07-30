# Getting Started with VESC Docker Environment

## Quick Start

### Running the Container

From the repository root, run the development container:

```bash
cd docker
./run.sh
```

This will start an interactive bash shell inside the container with your workspace mounted at `/home/developer/ws/src/project`.

### Building the Docker Image

To build or rebuild the Docker image:

```bash
./run.sh --build
```

### Running Commands Directly

Execute ROS 2 commands or other tools directly:

```bash
./run.sh -- ros2 topic list
./run.sh -- colcon build
./run.sh -- bash
```
