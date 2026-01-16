# ARPA Workspace - Docker Setup Guide for Gazebo Classic Simulation

This guide will help you run the ARPA system simulation using Docker with Gazebo Classic, even if you only have Ignition Gazebo on your host machine.

## Prerequisites

1. **Docker** installed on your system
2. **Docker Compose** (usually comes with Docker Desktop)
3. **X11 Forwarding** enabled (for GUI applications like Gazebo and RViz)

## Quick Start

### 1. Build the Docker Image

From the `arpa_ws` directory, build the Docker image:

```bash
cd /home/the2xman/arpa_ws
docker build -t arpa_system:latest -f Dockerfile .
```

**Note:** This will take 15-30 minutes on first build as it installs ROS 2 Humble, MoveIt2, Gazebo Classic, and builds all packages.

### 2. Allow X11 Forwarding

Allow Docker to connect to your X server:

```bash
xhost +local:docker
```

To make this permanent, add it to your `~/.bashrc`:
```bash
echo "xhost +local:docker" >> ~/.bashrc
```

### 3. Run the Container

You have two options:

#### Option A: Using Docker Compose (Recommended)

```bash
docker-compose up
```

Or run in detached mode:
```bash
docker-compose up -d
```

Then attach to the container:
```bash
docker exec -it arpa_system bash
```

#### Option B: Using Docker Run Directly

```bash
docker run -it --rm \
  --network host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd):/root/ros2_ws:rw \
  -v /dev/shm:/dev/shm \
  arpa_system:latest \
  bash
```

### 4. Inside the Container

Once inside the container, source the ROS 2 workspace:

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

### 5. Launch the Simulation

Launch the ARPA simulation with Gazebo Classic:

```bash
ros2 launch arpa_bringup arpa_sim.launch.py
```

This will:
- Start Gazebo Classic with an empty world
- Spawn the ARPA system (gantry + UR16e robot)
- Launch MoveIt2 for motion planning
- Start RViz2 for visualization
- Start the joint state broadcaster and trajectory controllers

### Launch Options

You can customize the launch with arguments:

```bash
# Launch without Gazebo GUI (headless)
ros2 launch arpa_bringup arpa_sim.launch.py gazebo_gui:=false

# Launch without RViz
ros2 launch arpa_bringup arpa_sim.launch.py launch_rviz:=false

# Launch without starting the joint controller
ros2 launch arpa_bringup arpa_sim.launch.py start_joint_controller:=false
```

## Alternative Launch Files

### Visualize Only (No Simulation)

To just visualize the robot model without Gazebo:

```bash
ros2 launch arpa_description arpa_display.launch.py
```

### MoveIt Only (No Gazebo)

To launch MoveIt2 without simulation:

```bash
ros2 launch arpa_bringup arpa_moveit.launch.py
```

## Troubleshooting

### Issue: Gazebo GUI doesn't appear

**Solution:** Make sure X11 forwarding is enabled:
```bash
xhost +local:docker
echo $DISPLAY  # Should show something like :0
```

### Issue: Permission denied errors

**Solution:** Run with proper permissions:
```bash
docker run --privileged ...
```

Or fix X11 permissions:
```bash
xhost +local:root
```

### Issue: Container can't find packages

**Solution:** Rebuild the workspace inside the container:
```bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Issue: Gazebo crashes or doesn't start

**Solution:** Check if Gazebo Classic is installed correctly:
```bash
gazebo --version
```

If it fails, rebuild the Docker image.

### Issue: "No space left on device"

**Solution:** Clean up Docker:
```bash
docker system prune -a
```

### Issue: Build fails during Docker build

**Solution:** Try building with more memory:
```bash
docker build --memory=4g -t arpa_system:latest -f Dockerfile .
```

## Useful Commands

### Stop the Container
```bash
# If using docker-compose
docker-compose down

# If using docker run
# Just press Ctrl+C or exit from the container
```

### Rebuild After Code Changes

If you modify code, you can rebuild inside the container:
```bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### View Running Containers
```bash
docker ps
```

### Access Container Shell
```bash
docker exec -it arpa_system bash
```

### View Logs
```bash
docker-compose logs -f
```

## Network Configuration

The Docker Compose file is configured with:
- `network_mode: "host"` - Container shares host network
- `ROS_DOMAIN_ID=21` - Make sure this matches your team
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` - CycloneDDS middleware

## GPU Support (Optional)

If you need GPU acceleration, the docker-compose.yaml includes NVIDIA runtime support. Make sure you have:
- NVIDIA drivers installed
- nvidia-docker2 installed

## Next Steps

1. Test the simulation is working
2. Try moving the robot using MoveIt2 in RViz
3. Experiment with different launch configurations
4. Connect real hardware (remove `use_fake_hardware` parameter)

## Getting Help

If you encounter issues:
1. Check the container logs: `docker-compose logs`
2. Verify ROS 2 installation: `ros2 doctor`
3. Check topic list: `ros2 topic list`
4. Verify robot description: `ros2 param get /robot_state_publisher robot_description`

