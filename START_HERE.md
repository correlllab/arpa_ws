# 🚀 ARPA Simulation - Docker Quick Start

**You need Gazebo Classic but only have Ignition Gazebo? No problem! Use Docker!**

## TL;DR - Get Started in 3 Steps

```bash
# 1. Build Docker image (takes 15-30 min, only first time)
docker build -t arpa_system:latest -f Dockerfile .

# 2. Allow GUI forwarding
xhost +local:docker

# 3. Run container and launch simulation
docker-compose up -d
docker exec -it arpa_system bash
# Inside container:
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
ros2 launch arpa_bringup arpa_sim.launch.py
```

## What This Does

1. **Builds a Docker container** with ROS 2 Humble, Gazebo Classic, MoveIt2, and all ARPA packages
2. **Runs the container** with X11 forwarding for GUI applications
3. **Launches the simulation** with:
   - Gazebo Classic (empty world)
   - ARPA system (gantry + inverted UR16e robot)
   - MoveIt2 for motion planning
   - RViz2 for visualization

## Files You Need

- ✅ `Dockerfile` - Updated with all required packages
- ✅ `docker-compose.yaml` - Already configured
- ✅ `QUICKSTART.md` - Detailed quick start guide
- ✅ `DOCKER_SETUP_GUIDE.md` - Comprehensive setup guide
- ✅ `run_simulation.sh` - Automated startup script

## Common Issues & Solutions

### ❌ Gazebo window doesn't appear
```bash
xhost +local:docker
echo $DISPLAY  # Should show :0
```

### ❌ "Container can't find packages"
Rebuild workspace inside container:
```bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### ❌ Build fails
Try with more memory:
```bash
docker build --memory=4g -t arpa_system:latest -f Dockerfile .
```

## What's Different from Your System?

- ✅ **Gazebo Classic** installed (not Ignition)
- ✅ All ROS 2 packages pre-installed
- ✅ Workspace already built
- ✅ All dependencies included
- ✅ Isolated environment (won't conflict with your system)

## Need More Help?

See `DOCKER_SETUP_GUIDE.md` for:
- Detailed troubleshooting
- Alternative launch configurations
- Network setup
- GPU support
- All available commands

---

**Happy Simulating! 🤖**

