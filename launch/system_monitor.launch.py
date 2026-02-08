# SPDX-FileCopyrightText: 2025 Daichi Utsugi
# SPDX-License-Identifier: BSD-3-Clause

import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. 指摘対策：引数 (監視パス) を外部から指定できるようにする
    target_path_arg = DeclareLaunchArgument(
        'target_path',
        default_value='/tmp', # デフォルト値
        description='監視するディレクトリのパス'
    )

    # 2. 配信ノードの設定
    # setup.py の entry_points で指定した名前 (system_monitor) に合わせる
    monitor_node = launch_ros.actions.Node(
        package='kadai2',
        executable='system_monitor',
        arguments=[LaunchConfiguration('target_path')], # 引数としてパスを渡す
    )

    # 3. 表示ノードの設定
    # executable名は setup.py の設定 (system_display) に合わせる
    display_node = launch_ros.actions.Node(
        package='kadai2',
        executable='system_display',
        output='screen' # コンソールにログを表示するために必須
    )

    return launch.LaunchDescription([
        target_path_arg,
        monitor_node,
        display_node
    ])
