# pb2025_sentry_ws

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

## 1. 项目介绍

深圳北理莫斯科大学北极熊战队 哨兵机器人 ROS 工作空间，包含串口通信、视觉、导航、决策、仿真模块。

## 2. Quick Start

### 2.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Ignition: [Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/)
- Install [OpenVINO 2023.3](https://docs.openvino.ai/2025/get-started/install-openvino.html?PACKAGE=OPENVINO_BASE&VERSION=v_2023_3_0&OP_SYSTEM=LINUX&DISTRIBUTION=APT)
- Install [small_icp](https://github.com/koide3/small_gicp):

    ```bash
    sudo apt install -y libeigen3-dev libomp-dev

    git clone https://github.com/koide3/small_gicp.git
    cd small_gicp
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
    sudo make install
    ```

### 2.2 Create Workspace

```bash
sudo apt install git-lfs
sudo pip install vcstool2
```

```bash
git clone https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_ws.git
```

```bash
cd pb2025_sentry_ws
```

```bash
vcs import --recursive . < dependencies.repos
```

> [!NOTE]
> `dependencies.repos` 文件已包含所有模块所依赖的仓库地址，无需手动查阅子模块的 README 手动克隆依赖。

### 2.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 10
```

> [!NOTE]
> 推荐使用 --symlink-install 选项来构建你的工作空间，因为 pb2025_sentry_ws 广泛使用了 launch.py 文件和 yaml 文件。这个构建参数会为那些非编译的源文件使用符号链接，这意味着当你调整参数文件时，不需要反复重建，只需要重新启动即可。

### 2.4 Running

将会运行串口通信、视觉、导航、决策模块，参数均读取自配置文件 [node_params](./src/pb2025_sentry_bringup/params/node_params.yaml)。

```bash
ros2 launch pb2025_sentry_bringup bringup.launch.py \
world:=<YOUR_PARAMS_FILE> \
use_rviz:=True
```

## 3. 常用调试启动命令

> [!NOTE]
> 请自行替换 `<YOUR_WORLD_NAME>` 为你的 map, pcd 的文件名；替换 `<YOUR_PARAMS_FILE>` 为你的配置文件的**绝对路径**，如 [node_params](./src/pb2025_sentry_bringup/params/node_params.yaml)。

### 3.1 子模块

Camera

```bash
ros2 launch hik_camera_ros2_driver hik_camera_launch.py params_file:=<YOUR_PARAMS_FILE>
```

Serial

```bash
ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py use_rviz:=True params_file:=<YOUR_PARAMS_FILE>
```

Vision

```bash
ros2 launch pb2025_vision_bringup rm_vision_reality_launch.py \
use_composition:=True \
use_rviz:=True \
params_file:=<YOUR_PARAMS_FILE>
```

Navigation

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=<YOUR_WORLD_NAME>  \
slam:=False
```

Behavior Tree

```bash
ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py params_file:=<YOUR_PARAMS_FILE>
```

### 3.2 Tools

Teleop gimbal

```bash
ros2 run teleop_gimbal_keyboard teleop_gimbal_keyboard
```

Convert .pcd to .pgm

```bash
ros2 launch pcd2pgm pcd2pgm_launch.py
```

Save map

```bash
ros2 run nav2_map_server map_saver_cli -f <YOUR_WORLD_NAME>
```

### 3.3 Rosbags

#### 3.3.1 Record

> [!TIP]
> 图像信息占用空间极大，若不需要录制图像信息，可删除 `front_industrial_camera/image` 和 `front_industrial_camera/camera_info`。建议降低 hik_camera_ros2_driver 参数中的相机帧率，以减少 rosbag 文件大小。

> [!NOTE]
> 本命令仅录制了传感器数据，没有直接录制 tf 信息，因此启动导航/视觉模块时，应设置 `use_robot_state_pub:=True`，以使用 joint_state 数据生成并发布整车 TF。

```bash
source install/setup.zsh

ros2 bag record -o sentry_$(date +%Y%m%d_%H%M%S) \
/serial/gimbal_joint_state \
/livox/imu \
/livox/lidar \
/front_industrial_camera/image \
/front_industrial_camera/camera_info \
--compression-mode file --compression-format zstd -d 30
```

#### 3.3.2 Play

> [!NOTE]
> 使用 `--clock` 参数，以发布 rosbag 中的时间戳到 `/clock` 话题。这意味着运行其他算法节点时，应设置 `use_sim_time:=True`。

```bash
ros2 bag play <YOUR_ROSBAG>.bag --clock
```

Example:

```bash
ros2 launch pb2025_sentry_bringup bringup.launch.py \
world:=<YOUR_WORLD_NAME> \
use_composition:=False \
use_rviz:=True \
use_sim_time:=True \
use_hik_camera:=False \
use_robot_state_pub:=True
```
