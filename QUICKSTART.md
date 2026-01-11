# Quick Start Guide - ARPA Simulation in Docker

This is a quick start guide to run the ARPA system simulation using Docker with Gazebo Classic.

## Prerequisites

- Docker installed
- Docker Compose installed (usually comes with Docker)
- X11 forwarding enabled (for GUI)

## Fastest Way to Get Started

### Step 1: Build the Docker Image (First Time Only)

```bash
cd /home/the2xman/arpa_ws
docker build -t arpa_system:latest -f Dockerfile .
```

**⏱️ This takes 15-30 minutes on first build**

### Step 2: Allow X11 Forwarding

```bash
xhost +local:docker
```

### Step 3: Run the Simulation

**Option A: Use the quick start script (Easiest)**
```bash
./run_simulation.sh
```

**Option B: Use Docker Compose**
```bash
docker-compose up -d
docker exec -it arpa_system bash
```

**Option C: Manual Docker Run**
```bash
docker run -it --rm \
  --network host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd):/root/ros2_ws:rw \
  arpa_system:latest \
  bash
```

### Step 4: Inside the Container

Once inside, source ROS 2 and launch the simulation:

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch arpa_bringup arpa_sim.launch.py
```

## What Should Happen

1. **Gazebo Classic** window should open showing an empty world
2. The **ARPA system** (gantry + UR16e robot) should spawn in Gazebo
3. **RViz2** window should open showing the robot model
4. You should see the robot in both Gazebo and RViz

## Troubleshooting

### Gazebo window doesn't appear
```bash
xhost +local:docker
echo $DISPLAY  # Should show :0 or similar
```

### Container can't find packages
```bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Build fails
```bash
# Clean and rebuild
docker system prune -a
docker build --no-cache -t arpa_system:latest -f Dockerfile .
```

## Common Commands

### Stop the simulation
Press `Ctrl+C` in the terminal running the launch file

### Exit container
```bash
exit
```

### Restart container
```bash
docker-compose restart
# or
docker start arpa_system
```

### View running containers
```bash
docker ps
```

### Check ROS topics
```bash
ros2 topic list
```

### Check robot state
```bash
ros2 topic echo /joint_states
```

## Launch Options

```bash
# Launch without Gazebo GUI (headless)
ros2 launch arpa_bringup arpa_sim.launch.py gazebo_gui:=false

# Launch without RViz
ros2 launch arpa_bringup arpa_sim.launch.py launch_rviz:=false

# Both options
ros2 launch arpa_bringup arpa_sim.launch.py gazebo_gui:=false launch_rviz:=false
```

## Next Steps

1. Test moving the robot using MoveIt2 in RViz
2. Try different launch configurations
3. Explore the robot description files
4. Experiment with motion planning

For more details, see `DOCKER_SETUP_GUIDE.md`

