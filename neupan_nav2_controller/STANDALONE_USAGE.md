# NeuPAN Nav2 Controller - 独立使用指南

## 🎯 简介

这是一个**完全独立的Nav2控制器插件**，用户只需编译成功即可在自己的Nav2参数配置中直接使用，无需复杂的依赖设置。

## 🚀 快速开始

### 1. 前置条件

```bash
# 确保有基本的Python依赖
sudo apt install python3-dev python3-pip
pip3 install numpy torch

# 确保有ROS2 Humble和Nav2
sudo apt install ros-humble-navigation2
```

### 2. 编译安装

```bash
# 在你的ROS2工作空间中
cd ~/your_ros2_ws/src
git clone <这个仓库>

# 编译
cd ~/your_ros2_ws
colcon build --packages-select neupan_nav2_controller
source install/setup.bash
```

### 3. 在Nav2中使用

在你的Nav2参数文件中，只需要修改controller部分：

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    
    FollowPath:
      plugin: "neupan_nav2_controller::NeuPANController"
      robot_type: "omni"  # 或 "diff", "acker"
      max_linear_velocity: 1.0
      max_angular_velocity: 1.5
      goal_tolerance: 0.25
      laser_topic: "/scan"
```

### 4. 启动导航

```bash
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=your_nav2_params.yaml
```

## ⚙️ 配置选项

### 机器人类型设置

| robot_type | 描述 | 控制输出 |
|------------|------|----------|
| `"omni"` | 全向轮机器人 | vx, vy |
| `"diff"` | 差速驱动机器人 | v, ω |
| `"acker"` | 阿克曼转向机器人 | v, δ |

### 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `robot_type` | string | "omni" | 机器人运动学类型 |
| `max_linear_velocity` | double | 0.5 | 最大线速度 (m/s) |
| `max_angular_velocity` | double | 1.0 | 最大角速度 (rad/s) |
| `goal_tolerance` | double | 0.25 | 目标容差 (m) |
| `laser_topic` | string | "/scan" | 激光雷达话题 |
| `dune_model_path` | string | "" | DUNE模型路径(可选) |
| `neupan_config_path` | string | "" | 配置文件路径(可选) |

## 🤖 自动配置机制

控制器具有**智能搜索功能**，会自动在以下位置查找NeuPAN模型和配置：

### 模型文件搜索路径：
- `./NeuPAN-main/example/model/{robot_type}_robot_default/model_5000.pth`
- `../NeuPAN-main/example/model/{robot_type}_robot_default/model_5000.pth`
- `~/NeuPAN-main/example/model/{robot_type}_robot_default/model_5000.pth`
- `~/ros2_ws/src/NeuPAN-main/example/model/{robot_type}_robot_default/model_5000.pth`

### 配置文件搜索路径：
- `./NeuPAN-main/example/non_obs/{robot_type}/planner.yaml`
- `../NeuPAN-main/example/non_obs/{robot_type}/planner.yaml`
- `~/NeuPAN-main/example/non_obs/{robot_type}/planner.yaml`
- `~/ros2_ws/src/NeuPAN-main/example/non_obs/{robot_type}/planner.yaml`

## 📦 建议的项目结构

```
your_ros2_ws/
├── src/
│   ├── neupan_nav2_controller/    # 这个控制器
│   ├── NeuPAN-main/               # NeuPAN算法库 (自动下载)
│   └── your_robot_package/        # 你的机器人包
└── ...
```

## 🛠️ 故障排除

### 问题1：编译失败
```bash
# 检查Python依赖
python3 -c "import numpy, torch; print('Dependencies OK')"

# 检查ROS2依赖
ros2 pkg list | grep nav2
```

### 问题2：运行时找不到NeuPAN模块
```bash
# 确保NeuPAN在搜索路径中
ls NeuPAN-main/neupan/neupan.py

# 或手动安装NeuPAN
cd NeuPAN-main && pip3 install -e .
```

### 问题3：找不到模型文件
```bash
# 检查模型文件是否存在
ls NeuPAN-main/example/model/*/model_5000.pth

# 或在参数中明确指定路径
dune_model_path: "/path/to/your/model.pth"
```

## 🎉 优势特点

- ✅ **零配置启动** - 无需额外的ROS2节点或复杂设置
- ✅ **智能路径搜索** - 自动找到模型和配置文件
- ✅ **完整Nav2集成** - 支持所有标准Nav2功能
- ✅ **多机器人支持** - 支持全向、差速、阿克曼三种类型
- ✅ **即插即用** - 编译完成即可使用

## 🔗 相关链接

- [NeuPAN算法原理](../NeuPAN-main/README.md)
- [Nav2官方文档](https://navigation.ros.org/)
- [ROS2 Humble文档](https://docs.ros.org/en/humble/)

---

**🌟 享受智能神经网络路径规划的强大功能！**