# 座標変換の実装（カメラ座標系 → ロボットベース座標系）

## 問題の背景

### 物理環境

- **ロボットベース座標系（base_link）**: ロボットの底面を原点とする。Z軸は鉛直上向き
- **机の高さ**: Z=1.0m（ロボットベースと同じ高さ）
- **対象物（キューブ）**: 一辺5cmの立方体、机の上（Z=1.0m）に配置
- **目標把持高さ**: Z=2.5cm（キューブの中心）

### カメラ配置

- **カメラ位置**: ロボットグリッパーに取り付けられている
- **カメラの姿勢**: 真下ではなく、斜め下を向いている
- **センサー値（Depth）**: カメラからキューブまでの直線距離 = 0.487m

### 発生していた問題

元のコード（`color_filtered_grasp_pcl.cpp`）では：

```cpp
// 問題のあるコード例（修正前）
geometry_msgs::msg::PoseStamped pose;
pose.header.frame_id = camera_frame;
pose.pose.position.x = centroid.x();  // カメラ座標系のまま
pose.pose.position.y = centroid.y();  // カメラ座標系のまま
pose.pose.position.z = centroid.z();  // カメラ座標系のまま
pose_pub_->publish(pose);  // base_linkへの変換なし
```

**問題点**:
1. カメラ座標系で計算された位置を**そのまま出力**している
2. カメラが斜めを向いているため、Z値が実際の高さと一致しない
3. 単純な「カメラ高さ + 距離」の計算では、物理的にありえない値になる
4. base_link座標系への変換が行われていない

## 解決策：TF2による座標変換

### ROS 2の座標系規約

ROS 2では**右手座標系**を使用：
- **ロボット座標系（base_link）**: X軸:前方、Y軸:左、Z軸:上
- **カメラ光学座標系**: X軸:右、Y軸:下、Z軸:前（カメラの視線方向）

### TF2とは

TF2（Transform Framework 2）は、ROS 2で異なる座標系間の変換を管理するライブラリです。

**主な機能**:
- 各フレーム間の座標変換（平行移動＋回転）を自動管理
- 時刻同期された変換の提供
- 複数フレーム間の連鎖変換の自動計算

**変換の数学的表現**:

座標系AからBへの変換は、4x4の同次変換行列で表現されます：

```
T_B←A = [R_B←A  | t_B←A]
        [0  0  0 |   1  ]
```

ここで：
- `R_B←A`: 3x3回転行列（座標系Aの向きを座標系Bの向きに変換）
- `t_B←A`: 3x1並進ベクトル（座標系Aの原点を座標系Bで表現）

点Pの座標変換：
```
P_B = R_B←A * P_A + t_B←A
```

### 実装の詳細

#### 1. 必要なヘッダーの追加

```cpp
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
```

#### 2. TF2バッファとリスナーの初期化

```cpp
// メンバ変数
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

// コンストラクタ内
tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
```

**役割**:
- `tf_buffer_`: 座標変換の履歴を保持するバッファ
- `tf_listener_`: TFトピックから変換情報を自動的に受信し、バッファに格納

#### 3. 座標変換関数の実装

```cpp
std::optional<geometry_msgs::msg::PoseStamped> transform_to_base_link(
  const geometry_msgs::msg::PoseStamped & pose_in_camera)
{
  // 複数のbase_link候補を試す（名前空間の違いに対応）
  const std::vector<std::string> candidate_targets = {
    "base_link",
    "crane_x7/base_link",
    "crane_x7/crane_x7_base_link"
  };

  for (const auto & target_frame : candidate_targets) {
    try {
      // TF2による座標変換（回転＋並進）
      geometry_msgs::msg::PoseStamped pose_in_base =
        tf_buffer_->transform(pose_in_camera, target_frame, tf2::durationFromSec(0.5));

      RCLCPP_INFO(get_logger(),
        "Transformed: camera[%.3f, %.3f, %.3f] → base[%.3f, %.3f, %.3f]",
        pose_in_camera.pose.position.x, pose_in_camera.pose.position.y, pose_in_camera.pose.position.z,
        pose_in_base.pose.position.x, pose_in_base.pose.position.y, pose_in_base.pose.position.z);

      return pose_in_base;
    } catch (const tf2::TransformException & ex) {
      continue;  // 次の候補を試す
    }
  }

  return std::nullopt;  // 変換失敗
}
```

**ポイント**:
- `tf_buffer_->transform()`が実際の座標変換を実行
- 回転行列と並進ベクトルを自動的に適用
- 複数の候補フレーム名を試すことで、名前空間の違いに対応

#### 4. 変換の適用

```cpp
// カメラ座標系での位置を作成
geometry_msgs::msg::PoseStamped pose_camera;
pose_camera.header.frame_id = "camera_link";
pose_camera.header.stamp = msg->header.stamp;
pose_camera.pose.position.x = centroid.x();
pose_camera.pose.position.y = centroid.y();
pose_camera.pose.position.z = centroid.z();
pose_camera.pose.orientation.x = q.x();
pose_camera.pose.orientation.y = q.y();
pose_camera.pose.orientation.z = q.z();
pose_camera.pose.orientation.w = q.w();

// base_linkフレームに変換
auto pose_base_opt = transform_to_base_link(pose_camera);
if (!pose_base_opt.has_value()) {
  RCLCPP_WARN(get_logger(), "Failed to transform pose to base_link");
  return;
}

// base_link座標系での位置を出力
pose_pub_->publish(pose_base_opt.value());
```

### カメラの斜め向きの考慮

TF2が自動的に以下を考慮します：

1. **カメラの回転**: カメラフレームとbase_linkフレーム間の回転行列が、カメラの傾きを正しく変換
2. **カメラの位置**: カメラの取り付け位置（平行移動）も自動的に適用
3. **距離の解釈**: カメラ座標系でのZ値（カメラからの距離）を、base_link座標系での実際の3D位置に変換

**具体例**:
```
カメラ座標系での位置: (0.1, 0.05, 0.487)  ← カメラからの斜めの距離
         ↓ TF2による変換（回転＋並進）
base_link座標系での位置: (0.35, -0.12, 1.025) ← 実際の空間位置
```

## まとめ

### 修正前の問題
- カメラ座標系の値をそのまま使用
- 斜めの距離を高さとして誤って扱う
- 物理的に矛盾する座標（机の下など）

### 修正後の利点
- TF2による正しい座標変換
- カメラの姿勢（傾き）を自動的に考慮
- base_link座標系での正確な3D位置を取得
- ロボットの動作計画で直接使用可能

## 参考情報

### color_sorting.cppでの実装

`color_sorting.cpp`では、同様のTF2による座標変換が既に実装されています：

```cpp
// color_sorting.cpp:514-546
geometry_msgs::msg::PointStamped p_cam, p_base;
p_cam.header = msg->header;
p_cam.point.x = camera_pos.x;
p_cam.point.y = camera_pos.y;
p_cam.point.z = camera_pos.z;

// 複数のフレーム候補を試す
for (const auto & src : candidate_sources) {
  for (const auto & tgt : candidate_targets) {
    try {
      p_cam.header.frame_id = src;
      p_base = tf_buffer_->transform(p_cam, tgt, tf2::durationFromSec(0.1));
      transformed = true;
      break;
    } catch (const tf2::TransformException &) {
      continue;
    }
  }
}
```

### デバッグコマンド

TF変換の確認：
```bash
# TFツリーを表示
ros2 run tf2_tools view_frames

# 特定のフレーム間の変換を表示
ros2 run tf2_ros tf2_echo base_link camera_link

# 利用可能なTFフレームのリスト
ros2 run tf2_ros tf2_monitor
```

---

**作成日**: 2025年11月28日
**関連ファイル**:
- `ros2/src/crane_x7_ros/crane_x7_examples/src/color_filtered_grasp_pcl.cpp`
- `ros2/src/crane_x7_ros/crane_x7_examples/src/color_sorting.cpp`
