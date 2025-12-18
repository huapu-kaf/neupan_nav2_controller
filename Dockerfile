# 使用ROS2 Humble官方镜像作为基础
FROM ros:humble

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# 设置工作目录
WORKDIR /ros2_ws

# 安装系统依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    python3-numpy \
    cmake \
    build-essential \
    git \
    curl \
    wget \
    vim \
    nano \
    htop \
    ros-humble-desktop \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# 安装Python依赖
RUN pip3 install --no-cache-dir \
    torch \
    torchvision \
    numpy \
    scipy \
    pyyaml \
    matplotlib

# 安装NeuPAN核心包 (模拟安装，如果实际存在的话)
# RUN pip3 install neupan

# 创建ROS2工作空间
RUN mkdir -p /ros2_ws/src

# 复制项目文件
COPY . /ros2_ws/src/neupan_nav2_controller/

# 更新rosdep数据库
RUN rosdep update

# 安装ROS2依赖
RUN cd /ros2_ws && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y"

# 构建项目
RUN cd /ros2_ws && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select neupan_nav2_controller --cmake-args -DCMAKE_BUILD_TYPE=Release"

# 创建测试脚本
COPY docker/test_script.sh /test_script.sh
RUN chmod +x /test_script.sh

# 设置ROS环境
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# 暴露必要的端口
EXPOSE 11311

# 设置入口点
ENTRYPOINT ["/bin/bash"]
CMD ["-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && /test_script.sh"]
