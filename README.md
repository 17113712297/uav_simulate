# uav_simulate

基于 PX4 + Gazebo + FAST-LIO 的室内无人机仿真功能包，支持 Livox Mid-360 激光雷达的仿真与定位。

---

## 目录

- [uav\_simulate](#uav_simulate)
  - [目录](#目录)
  - [功能简介](#功能简介)
  - [系统依赖](#系统依赖)
  - [工作空间结构](#工作空间结构)
  - [功能一：基于 Mid-360 的 FAST-LIO 激光定位仿真](#功能一基于-mid-360-的-fast-lio-激光定位仿真)
    - [步骤一：创建工作空间](#步骤一创建工作空间)
    - [步骤二：导入启动文件](#步骤二导入启动文件)
    - [步骤三：导入模型文件](#步骤三导入模型文件)
    - [步骤四：配置 setup.bash](#步骤四配置-setupbash)
    - [步骤五：安装 Mid-360 驱动与仿真插件](#步骤五安装-mid-360-驱动与仿真插件)
      - [1. Livox-SDK2](#1-livox-sdk2)
      - [2. livox\_ros\_driver2](#2-livox_ros_driver2)
      - [3. livox\_laser\_simulation（Mid-360 Gazebo 插件）](#3-livox_laser_simulationmid-360-gazebo-插件)
    - [步骤六：装载 Mid-360 雷达模型](#步骤六装载-mid-360-雷达模型)
      - [1. 下载并导入模型资源](#1-下载并导入模型资源)
      - [2. 修改模型文件](#2-修改模型文件)
      - [3. 挂载雷达到无人机](#3-挂载雷达到无人机)
      - [4. 调整 IMU 位置](#4-调整-imu-位置)
    - [步骤七：安装并配置 FAST-LIO](#步骤七安装并配置-fast-lio)
      - [1. 系统依赖](#1-系统依赖)
      - [2. 升级编译器至 GCC/G++ 9](#2-升级编译器至-gccg-9)
      - [3. 安装 Intel TBB 库](#3-安装-intel-tbb-库)
      - [4. 下载并修改 FAST-LIO](#4-下载并修改-fast-lio)
    - [步骤八：配置激光定位](#步骤八配置激光定位)
      - [1. 修改 livox\_points\_plugin](#1-修改-livox_points_plugin)
      - [2. 配置 PX4 使用激光定位](#2-配置-px4-使用激光定位)
      - [3. 里程计桥接节点](#3-里程计桥接节点)
    - [步骤九：启动与测试](#步骤九启动与测试)
  - [话题说明](#话题说明)
  - [注意事项](#注意事项)

---

## 功能简介

本功能包实现了以下能力：

- 在 Gazebo 中仿真搭载 Livox Mid-360 激光雷达的 Solo 无人机
- 通过 FAST-LIO 算法实现基于激光雷达的里程计与建图
- 将 FAST-LIO 输出的里程计桥接至 MAVROS，使 PX4 飞控使用激光定位（代替 GPS）
- 支持通过手机地面站（QGC）进行无线监控

---

## 系统依赖

| 依赖项 | 版本要求 |
|---|---|
| Ubuntu | 20.04 |
| ROS | Noetic |
| PX4 Firmware | v1.13+ |
| Gazebo | 11 |
| GCC / G++ | 9.0+ |
| XTDrone | — |

**APT 依赖：**

```bash
sudo apt-get install libgoogle-glog-dev libeigen3-dev libpcl-dev libyaml-cpp-dev
```

---

## 工作空间结构

```
catkin_ws/
└── src/
    ├── uav_simulate/              # 本功能包
    │   ├── launch/
    │   │   ├── fastlio_bringup.launch   # 一键启动入口
    │   │   ├── msg_MID360.launch        # 仿真环境启动
    │   │   └── px4.launch               # MAVROS 启动
    │   ├── models/
    │   │   ├── solo/                    # 无人机模型
    │   │   └── mid360/                  # 激光雷达模型
    │   ├── worlds/
    │   │   └── test_world.world         # 仿真世界
    │   └── scripts/
    │       └── lidar_transfer.py        # 里程计话题桥接节点
    ├── FAST_LIO/                        # FAST-LIO 算法包
    ├── livox_laser_simulation/          # Mid-360 Gazebo 仿真插件
    ├── livox_ros_driver2/               # Livox ROS 驱动
    └── Livox-SDK2/                      # Livox SDK（需 CATKIN_IGNORE）
```

---

## 功能一：基于 Mid-360 的 FAST-LIO 激光定位仿真

在 Gazebo 中搭建搭载 Livox Mid-360 激光雷达的 Solo 无人机仿真环境，通过 FAST-LIO 实现激光里程计，并将定位结果回传至 PX4 飞控，替代 GPS 实现室内自主定位。可通过 QGroundControl（QGC）手机地面站对无人机进行遥控监控。

---

### 步骤一：创建工作空间

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

---

### 步骤二：导入启动文件

将以下两个启动文件复制到 `catkin_ws/src/uav_simulate/launch/` 目录：

- `indoor1.launch`（来自 `PX4_Firmware/launch/`）
- `single_vehicle_spawn_xtd.launch`（来自 `PX4_Firmware/launch/`）

**修改 `indoor1.launch`（重命名为 `test_world.launch`）：**

1. **修改世界模型路径：**

    ```xml
    <!-- 修改前 -->
    <arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/indoor1.world"/>
    <!-- 修改后 -->
    <arg name="world" default="$(find uav_simulate)/worlds/test_world.world"/>
    ```

2. **修改无人机模型**（将 `iris` 和 `iris_stereo_camera` 替换为 `solo`）

3. **修改包含文件路径：**

    ```xml
    <!-- 修改前 -->
    <include file="$(find px4)/launch/single_vehicle_spawn_xtd.launch">
    <!-- 修改后 -->
    <include file="$(find uav_simulate)/launch/single_vehicle_spawn_xtd.launch">
    ```

4. **配置手机地面站通信**（电脑与手机需在同一局域网）：

    ```xml
    <!-- 修改前 -->
    <arg name="gcs_url" value=""/>
    <!-- 修改后（将 IP 替换为实际手机 IP） -->
    <arg name="gcs_url" value="udp://@192.168.1.105:14550"/>
    ```

    > 手机地面站设置：UDP 协议，端口 14550，服务器填写电脑 IP 地址。

**修改 `single_vehicle_spawn_xtd.launch`：**

将 SDF 模型路径修改为工作空间内的绝对路径：

```xml
<!-- 修改前 -->
$(find px4)/Tools/sitl_gazebo/models/$(arg sdf)/$(arg sdf).sdf
<!-- 修改后（根据实际路径修改） -->
/home/<your_username>/catkin_ws/src/uav_simulate/models/$(arg sdf)/$(arg sdf).sdf
```

---

### 步骤三：导入模型文件

从 PX4 Firmware 中拷贝所需文件：

| 来源 | 目标 |
|---|---|
| `PX4_Firmware/Tools/sitl_gazebo/models/solo/` | `uav_simulate/models/solo/` |
| `PX4_Firmware/Tools/sitl_gazebo/worlds/indoor1.world` | `uav_simulate/worlds/test_world.world` |

> 将 `indoor1.world` 重命名为 `test_world.world`。

---

### 步骤四：配置 setup.bash

将 PX4 路径追加到工作空间的 `setup.bash`，使 `roslaunch` 能找到 PX4 相关包：

```bash
echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/<your_username>/PX4_Firmware:/home/<your_username>/PX4_Firmware/Tools/sitl_gazebo' \
    >> ~/catkin_ws/devel/setup.bash
```

> ⚠️ 每次执行 `catkin_make` 后，`devel/setup.bash` 会被覆盖，需重新执行上述命令。

---

### 步骤五：安装 Mid-360 驱动与仿真插件

#### 1. Livox-SDK2

```bash
cd ~/catkin_ws/src
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j
sudo make install
# 避免被 catkin_make 编译
touch ~/catkin_ws/src/Livox-SDK2/CATKIN_IGNORE
```

> SDK 只需编译安装一次，重建工作空间后无需重新安装。

#### 2. livox_ros_driver2

```bash
cd ~/catkin_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
source /opt/ros/noetic/setup.sh
./build.sh ROS1
# 避免被 catkin_make 重复编译
touch ~/catkin_ws/src/livox_ros_driver2/CATKIN_IGNORE
```

> ⚠️ 必须使用 `./build.sh ROS1` 编译，不可使用 `catkin_make`。

#### 3. livox_laser_simulation（Mid-360 Gazebo 插件）

```bash
cd ~/catkin_ws/src
git clone https://github.com/fratopa/Mid360_simulation_plugin.git
mv Mid360_simulation_plugin/livox_laser_simulation .
rm -rf Mid360_simulation_plugin

cd ~/catkin_ws
source /opt/ros/noetic/setup.sh
catkin_make --pkg livox_laser_simulation -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

> 仅需将 `livox_laser_simulation` 文件夹放入 `src`，其余文件可删除。

---

### 步骤六：装载 Mid-360 雷达模型

#### 1. 下载并导入模型资源

参考教程：https://blog.csdn.net/m0_71983702/article/details/140365713

在 `uav_simulate/models/` 下创建 `mid360/` 文件夹，并放入以下文件：

```
mid360/
├── mid360.sdf         # 重命名自 Mid360.sdf
├── model.config
└── meshes/            # 来自 livox_mid40/meshes/
```

#### 2. 修改模型文件

**`model.config`**：将所有 `Mid360` 替换为 `mid360`。

**`mid360.sdf`（原 `Mid360.sdf`）**：
- 将所有 `Mid360` 替换为 `mid360`
- 修改 CSV 文件路径为相对路径：
    ```xml
    <csv_file_name>mid360-real-centr.csv</csv_file_name>
    ```
- 修改 mesh 路径为绝对路径（根据实际路径修改）：
    ```xml
    <uri>/home/<your_username>/catkin_ws/src/uav_simulate/models/mid360/meshes/livox_mid40.dae</uri>
    <uri>/home/<your_username>/catkin_ws/src/uav_simulate/models/mid360/meshes/mid360.dae</uri>
    ```

#### 3. 挂载雷达到无人机

在 `uav_simulate/models/solo/solo.sdf` 的 `<model>` 标签内添加：

```xml
<!-- 包含雷达模型，前倾 25°（0.4363 rad） -->
<include>
  <uri>/home/<your_username>/catkin_ws/src/uav_simulate/models/mid360</uri>
  <pose>0 0 0.05 0 0.4363 0</pose>
</include>
<joint name="lidar_joint" type="fixed">
  <child>mid360::livox_base</child>
  <parent>base_link</parent>
  <axis>
    <xyz>0 0 1</xyz>
    <limit>
      <upper>0</upper>
      <lower>0</lower>
    </limit>
  </axis>
</joint>
```

#### 4. 调整 IMU 位置

根据 FAST-LIO 外参配置，修改 `solo.sdf` 中的 IMU 位姿：

```xml
<!-- 修改前 -->
<link name='imu_link'>
    <pose>0 0 0 0 0 0</pose>
<!-- 修改后 -->
<link name='imu_link'>
    <pose>-0.016 -0.01829 0.09112 0 0 0</pose>
```

---

### 步骤七：安装并配置 FAST-LIO

参考：https://blog.csdn.net/m0_71983702/article/details/139634206

#### 1. 系统依赖

```bash
sudo apt-get update
sudo apt-get install libgoogle-glog-dev libeigen3-dev libpcl-dev libyaml-cpp-dev
```

#### 2. 升级编译器至 GCC/G++ 9

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt install gcc-9 g++-9
cd /usr/bin
sudo rm gcc g++
sudo ln -s gcc-9 gcc
sudo ln -s g++-9 g++
```

#### 3. 安装 Intel TBB 库

```bash
cd ~
wget https://github.com/uxlfoundation/oneTBB/releases/tag/2019_U8
tar zxvf oneTBB-2019_U8.tar.gz && rm oneTBB-2019_U8.tar.gz
cd oneTBB-2019_U8
cp build/linux.gcc.inc build/linux.gcc-9.inc
# 编辑 linux.gcc-9.inc，将 CPLUS 和 CONLY 修改为 g++-9 和 gcc-9
make compiler=gcc-9 stdver=c++17 tbb_build_prefix=my_tbb_build
```

```bash
sudo mkdir /usr/local/tbb-2019_U8
sudo cp -r include /usr/local/tbb-2019_U8/include
sudo cp -r build/my_tbb_build_release /usr/local/tbb-2019_U8/lib
sudo ln -s /usr/local/tbb-2019_U8/include/tbb /usr/local/include/tbb
sudo ln -s /usr/local/tbb-2019_U8/lib/libtbb.so.2 /usr/local/lib/libtbb.so
sudo ln -s /usr/local/tbb-2019_U8/lib/libtbbmalloc.so.2 /usr/local/lib/libtbbmalloc.so
sudo ln -s /usr/local/tbb-2019_U8/lib/libtbbmalloc_proxy.so.2 /usr/local/lib/libtbbmalloc_proxy.so
echo 'export LD_LIBRARY_PATH=/usr/local/tbb-2019_U8/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

#### 4. 下载并修改 FAST-LIO

```bash
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO/include && rm -rf ikd-Tree
git clone https://github.com/hku-mars/ikd-Tree.git
```

> `ikd-Tree` 目录下只保留 `ikd_Tree.cpp`、`ikd_Tree.h`、`README.md` 三个文件。

**修改 `package.xml` 和 `CMakeLists.txt`**：将所有 `livox_ros_driver` 依赖替换为 `livox_ros_driver2`。

**修改 `FAST_LIO/launch/mapping_mid360.launch`**：

```xml
<param name="use_sim_time" value="true"/>
```

**修改 `FAST_LIO/config/mid360.yaml`**：

```yaml
imu_topic: "/solo_0/mavros/imu/data"

extrinsic_T: [ 0.016, 0.01829, -0.04112 ]
extrinsic_R: [ 0.9063,  0, 0.4226,
               0,       1,    0,
               -0.4226,  0,  0.9063 ]
```

---

### 步骤八：配置激光定位

#### 1. 修改 livox_points_plugin

重写 `livox_laser_simulation/src/livox_points_plugin.cpp` 和对应头文件，实现同时发布 `PointCloud`、`PointCloud2`、`CustomMsg` 三种话题，且 `CustomMsg` 格式与 `livox_ros_driver2` 兼容。

参考：https://github.com/17113712297/livox_laser_simulation

#### 2. 配置 PX4 使用激光定位

修改 PX4 EKF2 参数，切换定位源为视觉/雷达里程计：

```bash
gedit ~/PX4_Firmware/build/px4_sitl_default/etc/init.d-posix/rcS
```

```bash
# 注释掉 GPS 定位
#param set EKF2_AID_MASK 1
# 启用视觉/雷达里程计定位
param set EKF2_AID_MASK 24

# 注释掉气压计高度
#param set EKF2_HGT_MODE 0
# 启用视觉高度
param set EKF2_HGT_MODE 3
```

每次修改参数后，重启仿真前需清除历史参数：

```bash
rm ~/.ros/eeprom/parameters*
rm -rf ~/.ros/sitl*
```

#### 3. 里程计桥接节点

在 `uav_simulate/scripts/lidar_transfer.py` 中实现节点，将 FAST-LIO 发布的 `/Odometry`（`nav_msgs/Odometry`）转发为 MAVROS 所需的 `/mavros/vision_pose/pose`（`geometry_msgs/PoseStamped`）。

赋予执行权限：

```bash
chmod +x scripts/lidar_transfer.py
```

---

### 步骤九：启动与测试

完成全部配置后，一键启动仿真与激光定位：

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch uav_simulate fastlio_bringup.launch
```

`fastlio_bringup.launch` 会依次启动：

1. `msg_MID360.launch` — Gazebo 仿真环境及无人机模型
2. `px4.launch` — MAVROS 节点
3. `mapping_mid360.launch` — FAST-LIO 激光里程计
4. `lidar_transfer.py` — 里程计桥接节点

启动后，打开手机上的 **QGroundControl（QGC）**，确认以下内容：

- 无人机连接成功，状态栏显示已连接
- 飞行模式可正常切换（如 Stabilized、Position 等）
- 可通过遥控器或 QGC 对无人机进行遥控操作

> 手机与电脑需在同一局域网下，QGC 设置：UDP 协议，端口 14550，服务器填写电脑 IP 地址。

---

## 话题说明

| 话题 | 类型 | 说明 |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 激光雷达点云 |
| `/livox/lidar` (CustomMsg) | `livox_ros_driver2/CustomMsg` | FAST-LIO 订阅的点云话题 |
| `/solo_0/mavros/imu/data` | `sensor_msgs/Imu` | IMU 数据 |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO 输出的里程计 |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | 送入 PX4 的视觉位姿 |

---

## 注意事项

- **路径**：所有包含绝对路径的文件（`solo.sdf`、`mid360.sdf`、`single_vehicle_spawn_xtd.launch` 等）在迁移到新环境时需要手动修改路径。
- **setup.bash**：每次 `catkin_make` 后需重新向 `devel/setup.bash` 追加 PX4 路径。
- **livox_ros_driver2**：只能用 `./build.sh ROS1` 编译，不可纳入 `catkin_make`。
- **Livox-SDK2**：只需安装一次，无需重复编译。
- **EKF2 参数**：修改 `rcS` 后，每次重启仿真前务必删除 `~/.ros/eeprom/parameters*` 和 `~/.ros/sitl*`。
- **局域网通信**：使用手机地面站时，手机与运行仿真的电脑需在同一局域网下。
