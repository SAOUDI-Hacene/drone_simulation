# Drone Simulation Project
هذا المشروع مخصص لمحاكاة درون باستخدام ROS 2 Jazzy.

## تعليمات التشغيل:
### Terminal 1
source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash
ros2 launch drone_simulation full.launch.py

### Terminal 2
source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash
python3 ~/drone_ws/src/drone_simulation/launch/drone_master.py
