# NeuPAN Nav2 控制器测试启动文件
# 使用 ROS2 Humble + Nav2 + NeuPAN Controller

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 获取包路径
    neupan_nav2_controller_dir = get_package_share_directory('neupan_nav2_controller')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 参数配置
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    # 重写 YAML 参数文件以包含正确的参数
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='顶级命名空间'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用仿真时间'),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='自动启动 Nav2 节点'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(neupan_nav2_controller_dir, 'config', 'nav2_params.yaml'),
            description='Nav2 参数文件路径'),

        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            description='是否使用组合节点'),

        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description='是否在失败时重启节点'),

        # 包含 Nav2 bringup 启动文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch',
                                                       'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                            'use_sim_time': use_sim_time,
                            'autostart': autostart,
                            'params_file': configured_params,
                            'use_composition': use_composition,
                            'use_respawn': use_respawn}.items()),

        # NeuPAN 控制器状态监控节点
        Node(
            package='neupan_nav2_controller',
            executable='test_neupan_plugin.py',
            name='neupan_monitor',
            output='screen',
            condition=IfCondition(PythonExpression(['True'])),
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/cmd_vel', '/neupan_cmd_vel'),
                ('/scan', '/scan')
            ]),
    ])