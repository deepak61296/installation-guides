# Complete ArduPilot + ROS2 + Gazebo Installation Guide

**System:** Ubuntu 22.04  
**ROS2 Version:** Humble  
**Gazebo Version:** Harmonic  
**Date:** December 28, 2025  
**Author:** Based on debugging session with Deepak

---

## Table of Contents
1. [Introduction](#introduction)
2. [System Requirements](#system-requirements)
3. [Installation Steps](#installation-steps)
4. [Issues Faced & Solutions](#issues-faced--solutions)
5. [Common Mistakes to Avoid](#common-mistakes-to-avoid)
6. [Final Working Configuration](#final-working-configuration)
7. [Testing & Verification](#testing--verification)
8. [References](#references)

---

## Introduction

This guide documents the complete process of installing ArduPilot SITL with ROS2 Humble and Gazebo Harmonic integration, including DDS (Data Distribution Service) support. This setup enables:

- Full ROS2 integration with ArduPilot via DDS
- Gazebo Harmonic physics simulation and visualization
- ROS2 topics (`/ap/*`) for sensor data
- ROS2 services for drone control (arm, mode switch, takeoff)
- Complete development environment for autonomous drone algorithms

### What Worked vs What Didn't

**✅ What Works:**
- ROS2 + ArduPilot DDS communication
- Gazebo visualization with physics sync
- ROS2 service-based control
- All sensor topics publishing

**❌ Initial Attempts That Failed:**
- Using eProsima's official Micro-XRCE-DDS-Gen (version mismatch)
- ArduPilot master branch without proper microxrceddsgen
- Manual SITL + Gazebo launches (protocol errors)

**🎯 Final Solution:**
- ArduPilot's fork of Micro-XRCE-DDS-Gen
- ardupilot_gz_bringup launch files
- Proper build order and dependencies

---

## System Requirements

### Hardware
- CPU: Multi-core processor (tested on 16 cores)
- RAM: 8GB minimum, 16GB recommended
- GPU: Optional but recommended for Gazebo visualization
- Storage: 20GB free space

### Software
- **OS:** Ubuntu 22.04 LTS
- **ROS2:** Humble Hawksbill
- **Gazebo:** Harmonic (gz-harmonic)
- **Python:** 3.10.12
- **GCC:** 11.4.0

---

## Installation Steps

### Step 1: Install ROS2 Humble

```bash
# Add ROS2 repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools -y

# Add to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Install Gazebo Harmonic

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic
sudo apt update
sudo apt install gz-harmonic -y

# Set environment variable
echo "export GZ_VERSION=harmonic" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Create ROS2 Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone required repositories
git clone https://github.com/ardupilot/ardupilot.git -b master
git clone https://github.com/micro-ROS/micro_ros_agent.git -b humble
git clone https://github.com/ArduPilot/ardupilot_gz.git -b main
git clone https://github.com/ArduPilot/ardupilot_gazebo.git -b main

# Initialize ArduPilot submodules
cd ardupilot
git submodule update --init --recursive
cd ~/ros2_ws
```

### Step 4: Install ROS-Gazebo Bridge

```bash
sudo apt install ros-humble-ros-gzharmonic -y
```

### Step 5: Install ArduPilot Dependencies

```bash
cd ~/ros2_ws/src/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

### Step 6: Install Micro-XRCE-DDS-Gen (CRITICAL!)

**🚨 IMPORTANT: Use ArduPilot's fork, NOT eProsima's official version!**

```bash
cd ~/ros2_ws

# Clone ArduPilot's fork (contains necessary patches)
git clone https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen

# Checkout the correct version
git checkout v4.7.0

# Initialize submodules
git submodule update --init --recursive

# Build
./gradlew assemble

# Add to PATH
export PATH=$HOME/ros2_ws/Micro-XRCE-DDS-Gen/scripts:$PATH
echo 'export PATH=$HOME/ros2_ws/Micro-XRCE-DDS-Gen/scripts:$PATH' >> ~/.bashrc

# Verify (should show "microxrceddsgen version: null")
microxrceddsgen -version
```

**Why ArduPilot's fork?**  
The official eProsima version (2.0.2/2.0.3) doesn't support the `-default-container-prealloc-size` argument that ArduPilot's build system requires. ArduPilot's fork v4.7.0 includes patches for compatibility.

### Step 7: Build ArduPilot with DDS

```bash
cd ~/ros2_ws/src/ardupilot

# Update submodules (fixes DroneCAN FlexDebug errors)
git submodule update --init --recursive --force

# Configure with DDS enabled (CAPITAL letters!)
./waf configure --board sitl --enable-DDS

# Build
./waf copter

# Verify DDS is enabled
ls -la build/sitl/libraries/AP_DDS/*.o
# Should see AP_DDS_Client.cpp.0.o and other DDS object files
```

**Common Error:** Using lowercase `--enable-dds` won't work on some versions!

### Step 8: Build ROS2 Workspace

```bash
cd ~/ros2_ws

# Source ROS2
source /opt/ros/humble/setup.bash

# Build micro_ros_agent first
colcon build --packages-select micro_ros_agent

# Build everything else
colcon build --packages-up-to ardupilot_gazebo ardupilot_gz_bringup

# Source workspace
source ~/ros2_ws/install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Issues Faced & Solutions

### Issue 1: "Incorrect protocol magic 0 should be 18458"

**Symptom:**
```
[Wrn] [ArduPilotPlugin.cc:1568] Incorrect protocol magic 0 should be 18458
```

**Root Cause:** Using `--model gazebo` instead of `--model JSON`

**Solution:**
```bash
# WRONG:
sim_vehicle.py -v ArduCopter -f gazebo-iris --model=gazebo

# CORRECT:
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON
```

ArduPilot uses JSON protocol (magic number 18458) to communicate with Gazebo, not the default "gazebo" model.

---

### Issue 2: Build Failure - "Unknown argument -default-container-prealloc-size"

**Symptom:**
```
ERROR<BadArgumentException>: Unknown argument -default-container-prealloc-size
```

**Root Cause:** Using eProsima's official Micro-XRCE-DDS-Gen instead of ArduPilot's fork

**Solution:** Use ArduPilot's fork (see Step 6)

---

### Issue 3: DroneCAN Compilation Error

**Symptom:**
```
error: 'dronecan_protocol_FlexDebug' has not been declared
```

**Root Cause:** DroneCAN submodule not properly initialized

**Solution:**
```bash
cd ~/ros2_ws/src/ardupilot
git submodule update --init --recursive --force
./waf distclean
./waf configure --board sitl --enable-DDS
./waf copter
```

The `--force` flag ensures all submodules are at correct commits.

---

### Issue 4: DDS_ENABLE Parameter Missing

**Symptom:** `param show DDS_ENABLE` returns "Unable to find parameter"

**Root Cause:** DDS wasn't actually compiled into the binary

**Verification:**
```bash
# Check if DDS library was built
ls build/sitl/libraries/AP_DDS/*.o

# If empty, DDS wasn't compiled
```

**Solution:** Ensure `--enable-DDS` (capital letters) was used during configure

---

### Issue 5: Gazebo Physics Not Syncing with ArduPilot

**Symptom:** 
- ArduPilot SITL shows drone at 10m altitude
- ROS2 topics show correct altitude
- Gazebo visualization shows drone on ground

**Root Cause:** Using separate SITL + Gazebo launches instead of integrated launch file

**Solution:** Use ardupilot_gz_bringup launch file:
```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

This properly coordinates ArduPilot, Gazebo, and micro_ros_agent.

---

### Issue 6: micro_ros_agent Not Found

**Symptom:**
```
Package 'micro_ros_agent' not found
```

**Root Cause:** Package not built or workspace not sourced

**Solution:**
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select micro_ros_agent
source install/setup.bash
```

---

### Issue 7: Multiple ArduPilot Installations Causing Conflicts

**Symptom:** `sim_vehicle.py` uses wrong ArduPilot directory

**Root Cause:** Multiple clones of ardupilot repository

**Solution:**
```bash
# Use full path to avoid ambiguity
~/ros2_ws/src/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter ...

# Or rename/remove old installations
mv ~/ardupilot ~/ardupilot_OLD
```

---

## Common Mistakes to Avoid

### 1. ❌ Using Wrong microxrceddsgen Version
**DON'T:** Clone from eProsima/Micro-XRCE-DDS-Gen  
**DO:** Clone from ardupilot/Micro-XRCE-DDS-Gen

### 2. ❌ Wrong Configure Command
**DON'T:** `./waf configure --board sitl --enable-dds` (lowercase)  
**DO:** `./waf configure --board sitl --enable-DDS` (uppercase)

### 3. ❌ Missing Submodule Updates
**DON'T:** Skip `git submodule update --init --recursive`  
**DO:** Always run submodule update, especially with `--force` flag

### 4. ❌ Wrong ArduPilot Model
**DON'T:** Use `--model gazebo`  
**DO:** Use `--model JSON` for Gazebo communication

### 5. ❌ Building Without Sourcing ROS2
**DON'T:**
```bash
colcon build  # Without sourcing
```
**DO:**
```bash
source /opt/ros/humble/setup.bash
colcon build
```

### 6. ❌ Using Manual Launches Instead of Launch Files
**DON'T:** Manually start Gazebo, ArduPilot, and micro_ros_agent separately  
**DO:** Use `ros2 launch ardupilot_gz_bringup iris_runway.launch.py`

### 7. ❌ Not Enabling DDS Before Testing
**DON'T:** Forget to `param set DDS_ENABLE 1; reboot`  
**DO:** Enable DDS immediately after SITL starts

---

## Final Working Configuration

### Directory Structure
```
~/ros2_ws/
├── src/
│   ├── ardupilot/              # ArduPilot source with DDS
│   ├── ardupilot_gz/           # Gazebo integration packages
│   ├── ardupilot_gazebo/       # Gazebo plugin
│   └── micro_ros_agent/        # DDS agent
├── install/                    # Built packages
├── build/                      # Build artifacts
└── Micro-XRCE-DDS-Gen/        # Code generator (ArduPilot fork)
```

### Environment Variables in ~/.bashrc
```bash
# ROS2 Humble
source /opt/ros/humble/setup.bash

# ROS2 workspace
source ~/ros2_ws/install/setup.bash

# Gazebo version
export GZ_VERSION=harmonic

# Micro-XRCE-DDS-Gen
export PATH=$HOME/ros2_ws/Micro-XRCE-DDS-Gen/scripts:$PATH
```

### Launch Command (THE MAGIC ONE!)
```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

This single command starts:
1. Gazebo Harmonic with iris_runway world
2. ArduPilot SITL with DDS enabled
3. micro_ros_agent for ROS2 communication
4. All necessary bridges and connections

---

## Testing & Verification

### Test 1: Check ROS2 Topics

```bash
# Open new terminal
source ~/ros2_ws/install/setup.bash
ros2 topic list
```

**Expected output:**
```
/ap/airspeed
/ap/battery
/ap/clock
/ap/cmd_gps_pose
/ap/cmd_vel
/ap/geopose/filtered
/ap/goal_lla
/ap/gps_global_origin/filtered
/ap/imu/experimental/data
/ap/joy
/ap/navsat
/ap/pose/filtered
/ap/rc
/ap/status
/ap/tf
/ap/tf_static
/ap/time
/ap/twist/filtered
/parameter_events
/rosout
```

### Test 2: Check ROS2 Services

```bash
ros2 service list | grep /ap
```

**Expected output:**
```
/ap/arm_motors
/ap/experimental/takeoff
/ap/mode_switch
/ap/param/get_parameters
/ap/param/set_parameters
```

### Test 3: Arm and Takeoff via ROS2

```bash
# Arm motors
ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"

# Switch to GUIDED mode (mode 4)
ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 4}"

# Takeoff to 10 meters
ros2 service call /ap/experimental/takeoff ardupilot_msgs/srv/Takeoff "{alt: 10.0}"
```

**Expected:** Drone should take off in Gazebo visualization AND ROS2 topics should show altitude increasing.

### Test 4: Monitor Drone State

```bash
# Watch position updates
ros2 topic echo /ap/pose/filtered

# Watch battery
ros2 topic echo /ap/battery

# Watch GPS
ros2 topic echo /ap/navsat
```

### Test 5: Check Gazebo-ArduPilot Connection

```bash
# In Gazebo terminal, should see:
# "New connection on SERIAL0"
# DDS initialization messages
# NO "Incorrect protocol magic" errors

# Check model position in Gazebo
gz model -m iris_with_gimbal -p
# Should show z > 0 when flying
```

---

## Troubleshooting

### Problem: DDS Not Connecting

**Check:**
```bash
# 1. Is DDS enabled?
# In MAVProxy: param show DDS_ENABLE
# Should be 1

# 2. Is micro_ros_agent running?
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019 -v4

# 3. Check for connection messages
# Should see "DDS: Initialization passed" in ArduPilot terminal
```

### Problem: Gazebo Not Showing Drone

**Check:**
```bash
# 1. Is Gazebo running?
gz topic -l

# 2. Is the world loaded?
gz world -l
# Should show "iris_runway"

# 3. Is the model spawned?
gz model -l
# Should show "iris_with_gimbal"
```

### Problem: Build Errors

```bash
# Clean everything and rebuild
cd ~/ros2_ws
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --packages-up-to ardupilot_gz_bringup
```

---

## Performance Tips

### 1. Reduce Gazebo Graphics Load
```bash
# Use -v4 flag sparingly (less verbose logging)
# Close unused sensor visualizations
# Reduce simulation real-time factor if needed
```

### 2. Speed Up Builds
```bash
# Use parallel jobs
colcon build -j8  # Use 8 cores

# Build only what changed
colcon build --packages-select <package_name>
```

### 3. Reduce Terminal Clutter
```bash
# Filter micro_ros_agent output
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019 2>&1 | grep -v "client_key"
```

---

## Development Workflow

### Daily Use

```bash
# 1. Start everything
cd ~/ros2_ws
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py

# 2. In new terminal, develop your ROS2 node
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/my_package
# ... develop, build, test
```

### Creating Your Own ROS2 Control Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from geometry_msgs.msg import PoseStamped

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Subscribe to pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self.pose_callback,
            10
        )
        
        # Service clients
        self.arm_client = self.create_client(ArmMotors, '/ap/arm_motors')
        self.mode_client = self.create_client(ModeSwitch, '/ap/mode_switch')
        self.takeoff_client = self.create_client(Takeoff, '/ap/experimental/takeoff')
        
    def pose_callback(self, msg):
        self.get_logger().info(f'Position: z={msg.pose.position.z:.2f}m')
    
    def arm_and_takeoff(self, altitude):
        # Arm
        arm_req = ArmMotors.Request()
        arm_req.arm = True
        self.arm_client.call_async(arm_req)
        
        # Guided mode
        mode_req = ModeSwitch.Request()
        mode_req.mode = 4
        self.mode_client.call_async(mode_req)
        
        # Takeoff
        takeoff_req = Takeoff.Request()
        takeoff_req.alt = altitude
        self.takeoff_client.call_async(takeoff_req)

def main():
    rclpy.init()
    controller = DroneController()
    controller.arm_and_takeoff(10.0)
    rclpy.spin(controller)

if __name__ == '__main__':
    main()
```

---

## Key Takeaways

### What We Learned

1. **ArduPilot's ecosystem requires specific versions** - Always use ArduPilot's forks when available
2. **Launch files are crucial** - Manual launches miss important coordination
3. **Submodules matter** - Always update recursively with --force
4. **Case sensitivity matters** - DDS vs dds, JSON vs json
5. **Multiple installations cause conflicts** - Keep one clean workspace

### Success Factors

1. ✅ Using ArduPilot's Micro-XRCE-DDS-Gen fork
2. ✅ Proper submodule initialization
3. ✅ Using ardupilot_gz_bringup launch files
4. ✅ Building in correct order
5. ✅ Enabling DDS with capital letters

---

## References

### Official Documentation
- [ArduPilot ROS2 Guide](https://ardupilot.org/dev/docs/ros2.html)
- [ardupilot_gazebo GitHub](https://github.com/ArduPilot/ardupilot_gazebo)
- [ardupilot_gz GitHub](https://github.com/ArduPilot/ardupilot_gz)
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [Gazebo Harmonic Docs](https://gazebosim.org/docs/harmonic)

### Community Resources
- [ArduPilot Discourse](https://discuss.ardupilot.org/)
- [ArduPilot Discord](https://ardupilot.org/discord)
- [ROS Answers](https://answers.ros.org/)

### GitHub Repositories
- [ArduPilot](https://github.com/ArduPilot/ardupilot)
- [Micro-XRCE-DDS-Gen (ArduPilot fork)](https://github.com/ardupilot/Micro-XRCE-DDS-Gen)
- [micro_ros_agent](https://github.com/micro-ROS/micro_ros_agent)

---

## Appendix: Build Logs

### Successful Build Output

**ArduPilot Configure:**
```
Enabled DDS                              : yes
Checking for program 'microxrceddsgen'  : /home/deepak/ros2_ws/Micro-XRCE-DDS-Gen/scripts/microxrceddsgen
```

**ArduPilot Build:**
```
[1479/1479] checking symbols build/sitl/bin/arducopter
'copter' finished successfully (34.544s)
```

**DDS Verification:**
```
$ ls build/sitl/libraries/AP_DDS/*.o
AP_DDS_Client.cpp.0.o
AP_DDS_ExternalControl.cpp.0.o
AP_DDS_External_Odom.cpp.0.o
AP_DDS_Serial.cpp.0.o
AP_DDS_Type_Conversions.cpp.0.o
```

**ROS2 Topics:**
```
$ ros2 topic list
/ap/airspeed
/ap/battery
/ap/clock
... (all 18+ topics)
```

---

## Version Information

This guide was tested with:
- **Ubuntu:** 22.04.3 LTS
- **ROS2:** Humble Hawksbill
- **Gazebo:** Harmonic (gz-harmonic 8.10.0)
- **ArduPilot:** Master branch (commit b60c0ad102)
- **Micro-XRCE-DDS-Gen:** v4.7.0 (ArduPilot fork)
- **Python:** 3.10.12
- **GCC:** 11.4.0

---

## License

This guide is provided as-is for educational purposes. ArduPilot, ROS2, and Gazebo are governed by their respective licenses.

---

## Acknowledgments

Special thanks to:
- ArduPilot development team for comprehensive documentation
- ROS2 community for excellent tooling
- Gazebo team for robust simulation platform
- Everyone who contributed to solving the issues documented here

---

**Last Updated:** December 28, 2025  
**Status:** ✅ Fully Working Configuration

**For questions or improvements, please open an issue on the relevant GitHub repository.**
