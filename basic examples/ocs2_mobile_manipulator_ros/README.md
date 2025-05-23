# OCS2 Mobile Manipulator

## 1. Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_mobile_manipulator_ros --symlink-install
```

## 2. Launch
### AgileX Piper
* Visualize Test
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch ocs2_mobile_manipulator_ros agileX_piper.launch.py visualize_only:=true
    ```
* Launch with Sim
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch ocs2_mobile_manipulator_ros agileX_piper.launch.py
    ```
### Franka Panda
* Visualize Test
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch ocs2_mobile_manipulator_ros franka.launch.py visualize_only:=true
    ```
* Launch with Sim
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch ocs2_mobile_manipulator_ros franka.launch.py
    ```

### Mabi-Mobile

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_mabi_mobile.launch.py
```

### Kinova Jaco2

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_kinova_j2n6.launch.py
```

### Willow Garage PR2

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_pr2.launch.py
```

### Clearpath Ridgeback with UR-5

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_ridgeback_ur5.launch.py 
```