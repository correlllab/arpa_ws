CLONE WITH
```
git clone --recurse-submodules https://github.com/correlllab/arpa_ws.git
```

**After editing robot description (e.g. `arpa_description/urdf/tool_holder.xacro`):**  
Rebuild so the sim and MoveIt use the new URDF:
```bash
colcon build --packages-select arpa_description
source install/setup.bash
```
Then relaunch. If you forget to rebuild, the launch will print a reminder.