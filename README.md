## kadai2
  本パッケージは、PCのリソース状況（CPU・メモリ・ディスク使用率）をROS 2トピックとして配信します。

## 構成ノード
  **system_monitor**
  システムのリソース状況を取得し、トピックとして配信します。
    - 配信するトピック

      | トピック名 | 型 | 内容 |
      | :--- | :--- | :--- |
      | `/cpu_usage`| `std_msgs/Float32` | CPU使用率(0.0 ~ 100.0 %) |
      | `/memory_usage`| `std_msgss/Float32` | メモリ使用率(0.0 ~ 100.0%) |
      | `/disk_usage` | `std_msgs/Float32` | ディスク使用率(0.0 ~ 100.0%) |
      | `/disk_gb` | `std_msgs/Float32` | ディスク使用量(GB) |

  **display_monitor**
  
