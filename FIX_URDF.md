# Fixed URDF File Issue

## Problem
The launch file was looking for `arpa_system.urdf.xacro` in `arpa_description/urdf/` but it only existed in `arpa_moveit_config/urdf/`.

## Solution
Copied the file to the correct location.

## Next Steps (Inside Docker Container)

Rebuild the `arpa_description` package so it installs the new file:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select arpa_description
source install/setup.bash
```

Then try launching again:

```bash
ros2 launch arpa_bringup arpa_sim.launch.py
```

