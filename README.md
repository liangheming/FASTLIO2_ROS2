# FASTLIO2 ROS2
## 主要工作
1. 重构[FASTLIO2](https://github.com/hku-mars/FAST_LIO) 适配ROS2
2. 添加回环节点，基于位置先验+ICP进行回环检测，基于GTSAM进行位姿图优化
3. 添加重定位节点，基于由粗到细两阶段ICP进行重定位
4. 增加一致性地图优化，基于[BLAM](https://github.com/hku-mars/BALM) (小场景地图) 和[HBA](https://github.com/hku-mars/HBA) (大场景地图)

## 环境依赖
1. Ubuntu 22.04
2. ROS2 Humble

## 编译依赖
```text
pcl
Eigen
sophus
gtsam
livox_ros_driver2
```

## 详细说明
### 1.编译 LIVOX-SDK2
```shell
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

### 2.编译 livox_ros_driver2
```shell
mkdir -r ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cd ws_livox/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble
```

### 3.编译 Sophus
```shell
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install
```

**新的Sophus依赖fmt，可以在CMakeLists.txt中添加add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)去除，否则会报错**


## 实例数据集
```text
链接: https://pan.baidu.com/s/1rTTUlVwxi1ZNo7ZmcpEZ7A?pwd=t6yb 提取码: t6yb 
--来自百度网盘超级会员v7的分享
```

## 部分脚本

### 1.激光惯性里程计 
```shell
ros2 launch fastlio2 lio_launch.py
ros2 bag play your_bag_file
```

### 2.里程计加回环
#### 启动回环节点
```shell
ros2 launch pgo pgo_launch.py
ros2 bag play your_bag_file
```
#### 保存地图
```shell
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: 'your_save_dir', save_patches: true}"
```

### 3.里程计加重定位

#### 原理说明

重定位模块用于在已知地图中定位机器人。系统架构如下：

```
┌─────────────────────────────────────────────────────────────┐
│                        重定位系统架构                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐  │
│  │   LiDAR      │    │    IMU       │    │  全局地图     │  │
│  │ /rslidar_pts │    │ /rslidar_imu │    │ global.pcd   │  │
│  └──────┬───────┘    └──────┬───────┘    └──────┬───────┘  │
│         │                   │                   │          │
│         └─────────┬─────────┘                   │          │
│                   ▼                             │          │
│         ┌─────────────────┐                     │          │
│         │    fastlio2     │                     │          │
│         │   (LIO里程计)    │                     │          │
│         └────────┬────────┘                     │          │
│                  │                              │          │
│    /fastlio2/lio_odom    /fastlio2/body_cloud   │          │
│                  │                              │          │
│                  ▼                              ▼          │
│         ┌───────────────────────────────────────┐          │
│         │           localizer                   │          │
│         │      (两阶段ICP重定位)                  │          │
│         │  1. 粗匹配: 0.25m分辨率, 5次迭代        │          │
│         │  2. 精匹配: 0.1m分辨率, 10次迭代        │          │
│         └────────────────┬──────────────────────┘          │
│                          │                                 │
│                          ▼                                 │
│                    TF: map → rslidar                       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

**核心原理：**
1. **FAST-LIO2 里程计**：融合LiDAR和IMU数据，输出局部里程计（相对于启动位置）
2. **两阶段ICP匹配**：
   - 粗匹配（Rough）：大分辨率体素滤波，快速收敛到大致位置
   - 精匹配（Refine）：小分辨率体素滤波，精确对齐
3. **TF发布**：localizer计算并发布 `map → local_frame` 变换

**TF树结构：**
```
map → odom → base_link → imu_link → rslidar
 │                                     ▲
 └─────────── localizer 发布 ───────────┘
```

#### 配置文件说明

**localizer.yaml** - 重定位节点配置：
```yaml
cloud_topic: /fastlio2/body_cloud   # 点云输入话题
odom_topic: /fastlio2/lio_odom      # 里程计输入话题
map_frame: map                       # 全局地图坐标系
local_frame: rslidar                 # 本地坐标系（会被odom消息的frame_id覆盖）
update_hz: 1.0                       # 定位更新频率

# 粗匹配参数
rough_scan_resolution: 0.25          # 扫描点云降采样分辨率
rough_map_resolution: 0.25           # 地图降采样分辨率
rough_max_iteration: 5               # 最大迭代次数
rough_score_thresh: 0.2              # 匹配分数阈值

# 精匹配参数
refine_scan_resolution: 0.1
refine_map_resolution: 0.1
refine_max_iteration: 10
refine_score_thresh: 0.1
```

**lio_airy.yaml** - FAST-LIO2配置（RoboSense Airy）：
```yaml
imu_topic: /rslidar_imu_data         # IMU话题
lidar_topic: /rslidar_points         # 点云话题
body_frame: base_link                # 机器人本体坐标系
world_frame: rslidar                 # LIO世界坐标系
imu_frame: imu_link                  # IMU坐标系（用于TF外参读取）
lidar_frame: rslidar                 # LiDAR坐标系（用于TF外参读取）
```

> **注意**：LiDAR-IMU外参优先从TF树读取（`imu_link → rslidar`），失败则使用配置文件中的 `r_il` 和 `t_il`

#### 使用方法

##### 方式一：使用Livox雷达（默认配置）
```shell
# 启动重定位节点
ros2 launch localizer localizer_launch.py

# 加载地图并设置初始位姿
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '/path/to/global_map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"

# 播放数据或启动实时雷达
ros2 bag play your_bag_file
```

##### 方式二：使用RoboSense Airy雷达
```shell
# 启动重定位节点（Airy配置，包含TF树和地图服务器）
ros2 launch localizer localizer_airy_launch.py

# 可选：指定地图路径
ros2 launch localizer localizer_airy_launch.py map_pcd:=/path/to/your_map.pcd

# 如需手动设置初始位姿
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '/path/to/global_map.pcd', x: 1.0, y: 2.0, z: 0.0, yaw: 1.57, pitch: 0.0, roll: 0.0}"
```

##### 检查重定位状态
```shell
# 检查是否定位成功
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{code: 0}"
# 返回 valid: true 表示定位成功
```

##### 动态加载新地图
```shell
# 通过话题请求加载新地图（map_server支持）
ros2 topic pub --once /map_request/pcd std_msgs/msg/String "{data: '/path/to/new_map.pcd'}"
```

#### 相关话题和服务

| 名称 | 类型 | 说明 |
|------|------|------|
| `/localizer/relocalize` | Service | 加载地图并设置初始位姿 |
| `/localizer/relocalize_check` | Service | 检查定位是否成功 |
| `/globalmap` | Topic | 全局地图点云（用于RViz可视化） |
| `/localizer/map_cloud` | Topic | localizer内部地图点云 |
| `/fastlio2/lio_odom` | Topic | LIO里程计输出 |
| `/fastlio2/body_cloud` | Topic | 机器人本体坐标系下的点云 |

#### RViz可视化配置

1. **Fixed Frame**: `map`
2. **添加显示项**:
   - PointCloud2: `/globalmap` (全局地图)
   - PointCloud2: `/fastlio2/body_cloud` (当前扫描)
   - TF: 显示坐标系关系
   - Path: `/fastlio2/lio_path` (轨迹)

### 4.一致性地图优化
#### 启动一致性地图优化节点
```shell
ros2 launch hba hba_launch.py
```
#### 调用优化服务
```shell
ros2 service call /hba/refine_map interface/srv/RefineMap "{"maps_path": "your maps directory"}"
```
**如果需要调用优化服务，保存地图时需要设置save_patches为true**

## 特别感谢
1. [FASTLIO2](https://github.com/hku-mars/FAST_LIO)
2. [BLAM](https://github.com/hku-mars/BALM)
3. [HBA](https://github.com/hku-mars/HBA)
## 性能相关的问题
该代码主要使用timerCB作为频率触发主函数，由于ROS2中的timer、subscriber以及service的回调实际上运行在同一个线程上，在电脑性能不是好的时候，会出现调用阻塞的情况，建议使用线程并发的方式将耗时的回调独立出来(如timerCB)来提升性能

