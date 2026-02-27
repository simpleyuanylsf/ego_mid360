# EGO-Planner 无人机自主导航系统 (MID360 + FAST-LIO 版本)

基于 MID360 LiDAR 和 FAST-LIO 的无人机自主导航框架，使用 EGO-Planner 进行局部轨迹规划。

> ⚠️ **注意：本仓库为非 FAST-LIO 高频里程计版本**

## 硬件需求

- 无人机平台 + PX4 飞控
- Livox MID360 LiDAR
- 机载计算机（NUC/Jetson 等）

## 软件依赖

```bash
# ROS 依赖
sudo apt-get install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-tf

# Livox SDK
# 参考: https://github.com/Livox-SDK/Livox-SDK2

# FAST-LIO
# 参考: https://github.com/hku-mars/FAST_LIO
```

## 编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 运行步骤

按顺序在5个终端中启动以下节点：

### 终端 1 - 启动 MID360 ROS 驱动
```bash
roslaunch livox_ros_driver2 msg_MID360.launch
```

### 终端 2 - 启动 FAST-LIO 里程计
```bash
roslaunch fast_lio mapping_mid360.launch
```

### 终端 3 - 启动 MAVROS
```bash
roslaunch mavros px4.launch 
```
> 根据实际连接修改串口参数

### 终端 4 - 启动 EGO-Planner 规划器
```bash
roslaunch ego_planner single_run_in_exp.launch
```

### 终端 5 - 启动控制器
```bash
roslaunch px4ctrl run_ctrl.launch
```

## 使用方法

1. 确保所有节点正常启动且无报错
2. 在 RViz 中点击 **2D Nav Goal** 按钮选择目标点
3. 无人机将自动规划并执行轨迹

## 配置文件说明

| 配置文件 | 路径 | 说明 |
|---------|------|------|
| EGO-Planner 参数 | `src/planner/plan_manage/launch/advanced_param_exp.xml` | 地图大小、速度限制等 |
| PX4Ctrl 参数 | `src/realflight_modules/px4ctrl/config/ctrl_param_fpv.yaml` | 控制增益、限制参数 |

### 常用参数调整

**EGO-Planner** (`single_run_in_exp.launch`):
```xml
<arg name="odom_topic" value="/Odometry" />        <!-- FAST-LIO 里程计话题 -->
<arg name="cloud_topic" value="/cloud_registered" /> <!-- MID360 点云话题 -->
<arg name="max_vel" value="0.4" />                  <!-- 最大速度 m/s -->
<arg name="max_acc" value="0.75" />                 <!-- 最大加速度 m/s² -->
```

**PX4Ctrl** (`run_ctrl.launch`):
```xml
<remap from="~odom" to="/Odometry" />     <!-- 里程计输入 -->
<remap from="~cmd" to="/position_cmd" />  <!-- 控制指令输出 -->
```

## 话题映射

```
MID360 Driver (/livox/lidar)
        ↓
FAST-LIO (/Odometry, /cloud_registered)
        ↓
EGO-Planner (/position_cmd)
        ↓
PX4Ctrl → MAVROS → PX4
```

## 坐标系说明

- **world**: 世界坐标系（规划器使用）
- **camera_init**: FAST-LIO 初始坐标系
- **body**: 无人机机体坐标系
- **map**: PX4 地图坐标系

## 故障排查

| 问题 | 解决方案 |
|------|---------|
| FAST-LIO 漂移 | 检查 MID360 时间同步，确保静止初始化 |
| 规划器无输出 | 确认里程计话题 `/Odometry` 正常发布 |
| 控制无响应 | 检查 MAVROS 连接状态，`rostopic echo /mavros/state` |
| 轨迹跟踪差 | 调整 PX4Ctrl 中的控制增益参数 |

## 项目结构

```
src/
├── planner/              # EGO-Planner 规划器
│   ├── plan_manage/      # 主规划节点
│   ├── plan_env/         # 地图构建
│   ├── path_searching/   # 路径搜索
│   └── bspline_opt/      # B样条优化
├── ego_planner_ctrl/     # FUEL 全局规划控制
└── realflight_modules/   # 实飞模块
    └── px4ctrl/          # PX4 控制接口
```

## 致谢

- [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner)
- [FAST-LIO](https://github.com/hku-mars/FAST_LIO)
- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
