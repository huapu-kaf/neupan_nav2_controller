# NeuPAN Nav2 Controller Docker 测试指南

本文档说明如何使用Docker对NeuPAN Nav2 Controller进行完整的功能测试。

## 📋 测试环境要求

### Windows 系统要求
- Windows 10/11 
- Docker Desktop for Windows (已安装并运行)
- PowerShell 5.1+ 或 Windows Terminal
- 至少 4GB 可用内存
- 至少 10GB 可用磁盘空间

### Linux 系统要求  
- Ubuntu 18.04+ 或其他支持Docker的Linux发行版
- Docker Engine 20.10+
- Docker Compose v2.0+
- 至少 4GB 可用内存
- 至少 10GB 可用磁盘空间

## 🚀 快速开始

### Windows PowerShell

```powershell
# 1. 进入项目目录
cd "C:\Users\86185\Desktop\workspace\NeuPAN\neupan_nav2_controller"

# 2. 运行基础测试
.\docker_test.ps1

# 3. 运行交互式测试（可以进入容器调试）
.\docker_test.ps1 -Interactive

# 4. 运行带仿真环境的测试
.\docker_test.ps1 -WithSimulation

# 5. 仅构建Docker镜像
.\docker_test.ps1 -BuildOnly

# 6. 清理Docker资源
.\docker_test.ps1 -Clean
```

### Linux Bash

```bash
# 1. 进入项目目录
cd /path/to/neupan_nav2_controller

# 2. 运行基础测试
docker-compose up

# 3. 运行交互式测试
docker-compose run --rm neupan_nav2_controller_test /bin/bash

# 4. 运行带仿真环境的测试
docker-compose --profile simulation up

# 5. 仅构建Docker镜像
docker-compose build

# 6. 清理Docker资源
docker-compose down --rmi all --volumes --remove-orphans
```

## 📊 测试内容

Docker测试将验证以下功能：

### ✅ 基础功能测试
1. **ROS2环境验证** - 检查ROS2 Humble是否正确安装
2. **项目构建验证** - 验证C++代码是否成功编译
3. **插件注册验证** - 检查插件是否正确注册到Nav2
4. **Python依赖验证** - 验证NumPy、PyTorch等依赖是否可用
5. **模型文件验证** - 检查神经网络模型文件是否存在
6. **配置文件验证** - 验证插件配置文件格式是否正确

### 🔧 高级功能测试
7. **控制器加载测试** - 尝试加载控制器到Nav2环境
8. **项目测试脚本** - 运行项目自带的测试脚本
9. **性能监控** - 监控内存和CPU使用情况

## 📁 测试结果

测试完成后，结果将保存在 `test_results/` 目录中：

- `test_report.md` - 完整的测试报告
- `ros2_version.txt` - ROS2版本信息  
- `plugin_status.txt` - 插件注册状态
- `python_deps.txt` - Python依赖检查结果
- `model_files.txt` - 模型文件列表
- `plugin_config.txt` - 插件配置内容
- `plugin_registration_test.txt` - 插件注册测试结果
- `controller_test.txt` - 控制器加载测试结果

## 🐛 故障排除

### 常见问题

#### 1. Docker构建失败
```bash
# 检查Docker是否运行
docker --version
docker info

# 清理Docker缓存
docker system prune -f
docker builder prune -f
```

#### 2. 权限问题 (Linux)
```bash  
# 将用户添加到docker组
sudo usermod -aG docker $USER
# 重新登录或运行
newgrp docker
```

#### 3. 内存不足
```bash
# 检查可用内存
free -h
# 关闭其他应用程序或增加虚拟内存
```

#### 4. 网络问题
```bash
# 检查网络连接
ping google.com
# 配置Docker使用代理（如果需要）
```

### Windows特定问题

#### PowerShell执行策略
```powershell
# 如果脚本无法执行，运行：
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

#### Docker Desktop未运行
- 确保Docker Desktop正在运行
- 检查系统托盘中的Docker图标
- 重启Docker Desktop

### Linux特定问题

#### Docker Compose版本
```bash
# 升级到最新版本
sudo apt update
sudo apt install docker-compose-plugin
```

## 🔍 详细测试步骤

### 步骤1: 构建Docker镜像
- 下载ROS2 Humble基础镜像
- 安装系统依赖（Python、编译工具等）
- 安装Python依赖（PyTorch、NumPy等）
- 复制项目文件到容器
- 使用colcon构建ROS2包

### 步骤2: 运行基础测试
- 验证ROS2环境完整性
- 检查项目编译结果
- 测试插件注册状态
- 验证Python依赖可用性

### 步骤3: 运行高级测试
- 尝试启动控制器服务器
- 执行项目自定义测试脚本
- 验证模型和配置文件

### 步骤4: 生成报告
- 汇总所有测试结果
- 生成Markdown格式的测试报告
- 标记成功和失败的测试项

## 📈 性能基准

在标准硬件配置下的预期性能：

| 指标 | 预期值 | 说明 |
|------|--------|------|
| 构建时间 | 5-15分钟 | 取决于网络和硬件 |
| 测试时间 | 1-3分钟 | 基础功能测试 |
| 内存使用 | 2-4GB | Docker容器运行时 |
| 磁盘占用 | 8-12GB | 包含所有镜像和缓存 |

## 🤝 贡献

如果您发现测试脚本的问题或有改进建议：

1. 创建Issue描述问题
2. 提交Pull Request
3. 更新文档

## 📚 相关资源

- [Docker官方文档](https://docs.docker.com/)
- [ROS2 Humble文档](https://docs.ros.org/en/humble/)
- [Nav2文档](https://navigation.ros.org/)
- [NeuPAN项目主页](https://github.com/hanruihua/neupan)

---

**💡 提示**: 首次运行可能需要较长时间来下载Docker镜像和编译代码，请耐心等待。
