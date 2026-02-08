## kadai2
  本パッケージに含まれるノードは、PCのリソース状況（CPU・メモリ・ディスク使用率）をROS 2トピックとして配信します。
  
  [![test](https://github.com/snake0227/kadai2/actions/workflows/test.yml/badge.svg)](https://github.com/snake0227/kadai2/actions/workflows/test.yml)

## 構成ノード
**system_monitor**

システムのリソース状況を取得し、トピックとして配信します。
- 配信するトピック
  | トピック名 | 型 | 内容 |
  | :--- | :--- | :--- |
  | `/cpu_usage`| `std_msgs/Float32` | CPU使用率(0.0 ~ 100.0 %) |
  | `/memory_usage`| `std_msgs/Float32` | メモリ使用率(0.0 ~ 100.0%) |
  | `/disk_usage` | `std_msgs/Float32` | ディスク使用率(0.0 ~ 100.0%) |
  | `/disk_gb` | `std_msgs/Float32` | ディスク使用量(GB) |

**display_monitor**

受信したリソース情報を整形して標準出力に表示します。
  - サブスクライブするトピック
    `/cpu_usage`, `/memory_usage`, `/disk_usage`, `/disk_gb`

## 使用方法
  **ノードを起動**
  ```bash
  ros2 launch kadai2 system_monitor.launch.py target_path:=/tmp
  ```

  **実行結果の例**
  ```text
  [system_monitor-1] [INFO] [1769055213.629795347] [system_monitor]: Monitoring path: /tmp
  [system_monitor-1] [INFO] [1769055215.626859715] [system_monitor]: Published metrics - CPU: 0.8%, Memory: 5.0%, Disk: 6.601469952GB,0.61%
  [system_display-2] [INFO] [1769055215.627103265] [system_display]: [RECEIVED] CPU Usage: 0.800000011920929%
  [system_display-2] [INFO] [1769055215.627298132] [system_display]: [RECEIVED] Disk Usage: 0.6%
  [system_display-2] [INFO] [1769055215.627629040] [system_display]: [RECEIVED] Memory Usage: 5.0%
  [system_display-2] [INFO] [1769055215.627773298] [system_display]: [RECEIVED] Disk Raw Usage: 6.601469993591309 GB
  ```
## 必要なソフトウェア
  - **OS**: Ubuntu 24.04(開発)/ Ubuntu 22.04(テスト)
  - **ROS2**: Jazzy(開発)/ Humble(テスト)
  - **Python**: 3.10以上
  - **外部ライブラリ**: `psutil`

## ライセンス
  - このソフトウェアは、BSD3条項ライセンスの下で再頒布及び仕様が許可されます。
  - このパッケージのコードの一部は、以下の講義資料(CC-BY-SA 4.0 by Ryuichi Ueda)を参考にして作成されています。
      - https://github.com/ryuichiueda/slides_marp/tree/master/robosys2025
  - © Snake0227
