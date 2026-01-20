# Repo Description

ROS 2 Humble development workspace running inside a Docker container, intended for robotics development with custom interfaces and messages.
This repository includes ROS 2 packages, custom message definitions, and a Docker-based development environment.

---

## Repository structure

```
mec_ws/
├── src/
│   ├── Actuator-dynamics-identification/   # Submodule (external repository)
│   └── pi3hat_moteus_int_msgs/              # Custom ROS 2 message package
├── Dockerfile
├── docker-compose.yml
├── entrypoint.sh
└── README.md
```

---

## Submodules

**Git submodule**:

* **Actuator-dynamics-identification**
  👉 https://github.com/CentroEPiaggio/Actuator-dynamics-identification

After cloning this repository, make sure to initialize and update submodules:

```bash
git submodule update --init --recursive
```

---

## `pi3hat_moteus_int_msgs` package

The package `pi3hat_moteus_int_msgs` defines **custom ROS 2 message types** used for interfacing a Raspberry Pi + pi3hat with moteus motor controllers.

**Location**

```
src/pi3hat_moteus_int_msgs
```

**Message definitions**

```
msg/
├── FeetPos.msg
├── JointsCommand.msg
├── JointsStates.msg
├── OmniMulinexCommand.msg
├── PacketPass.msg
└── QuadFootState.msg
```

These messages are meant to:

* Represent joint commands and joint states
* Exchange structured data between low-level motor control and higher-level ROS 2 nodes
* Provide a clean ROS 2 interface for pi3hat + moteus-based systems

---

## Requirements

* Docker
* Docker Compose
* Linux with X11 (for GUI tools such as `rqt` and PlotJuggler)

---

## Docker-based development environment

The Docker image is based on **ROS 2 Humble Desktop** and includes:

* `colcon`, `rosdep`, `vcstool`
* `ros2_control` and controllers
* URDF / xacro tools
* GUI tools: `rqt`, PlotJuggler (+ ROS plugin)
* Non-root user: `ros`
* Workspace: `/home/ros/colcon_ws`

---

## Getting started

### 1. Clone the repository

```bash
git clone https://github.com/lo-imperatore/mec_ws.git
cd mec_ws
git submodule update --init --recursive
```

---

### 2. Allow Docker to access X11 (GUI)

On the host machine:

```bash
xhost +local:docker
```

---

### 3. Build and start the container

From the repository root:

```bash
docker compose up --build
```

Or run in detached mode:

```bash
docker compose up --build -d
```

---

### 4. Open a shell inside the container

If running detached:

```bash
docker compose exec ros2-dev bash
```

You will be logged in as user `ros` in:

```
/home/ros/colcon_ws
```

---

## Volumes and mounts

* `./src` → `/home/ros/colcon_ws/src`
* `/tmp/.X11-unix` → GUI forwarding
* `${HOME}/.ssh` → SSH access from inside the container (read-only)
* `ros_home` → persistent `/home/ros`

---

## ROS 2 networking

Default configuration (can be changed in `docker-compose.yml`):

```yaml
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_LOCALHOST_ONLY=0
```

Ensure all ROS 2 machines use the same `ROS_DOMAIN_ID`.

---

## Stopping the environment

```bash
docker compose down
```

Remove containers and volumes:

```bash
docker compose down -v
```

---

## Notes

* Custom messages in `pi3hat_moteus_int_msgs` must be built with `colcon` before use.
* SSH access is required to connect from inside the Docker container to a Raspberry Pi running the low-level control stack.

---

## License

Add license information here.
