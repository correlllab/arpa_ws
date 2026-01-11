# Fix Issues Inside Docker Container

## You're Inside! ✅

The prompt `root@FastInfoSlab:~/ros2_ws#` confirms you're in the Docker container.

## Issues to Fix

1. **Missing install directory** - The workspace needs to be built
2. **Missing CycloneDDS** - RMW middleware not installed

## Solution: Run These Commands Inside the Container

```bash
# 1. Install CycloneDDS
apt update
apt install -y ros-humble-rmw-cyclonedds-cpp

# 2. Build the workspace
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 3. Source the workspace
source install/setup.bash

# 4. Now launch simulation
ros2 launch arpa_bringup arpa_sim.launch.py
```

## Alternative: Quick Test Without Building

If you just want to test if things work, you can use the pre-built packages:

```bash
source /opt/ros/humble/setup.bash
apt install -y ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp  # Use FastRTPS instead
ros2 launch arpa_bringup arpa_sim.launch.py
```

But building is recommended for full functionality.

