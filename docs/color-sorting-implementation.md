# 色分別バナナ仕分けデモの実装

## プログラム構成

```
crane_x7_examples/
├── src/color_sorting.cpp              # メインプログラム
├── launch/color_sorting.launch.py     # 起動ファイル
├── CMakeLists.txt                     # ビルド設定
└── package.xml                        # 依存関係定義
```

## 実装の特徴

### 1. ビジョンベースの物体検出

- **OpenCV**による HSV 色空間での色検出
- **RealSense D435**カメラによる RGB-D センシング
- 深度情報を使った 3D 位置推定
- 中央値フィルタによる深度ノイズ除去

### 2. 複数物体の一括検出

- バッチスキャン方式で視野内の全ての物体を検出
- 重複検出の自動除外（距離閾値: 5cm）
- 作業エリア内の物体のみをターゲット化

### 3. TF2 による座標変換

- カメラ座標系 → ロボットベース座標系の変換
- 複数フレーム候補への自動フォールバック
- 名前空間/プレフィックスの違いに対応

### 4. 動的な物体スポーン

- Gazebo シミュレーター内での物体生成
- 色ごとに異なるマテリアル設定
- SDF フォーマットでの動的モデル生成

### 5. 精度向上の工夫

- 近距離再検出による位置補正
- デカルト空間での直線軌道計画
- MoveIt の再プランニング機能の活用

## 色検出アルゴリズム

### HSV 色空間での検出

OpenCV の `cv::inRange()` を使用して、各色の HSV 範囲でマスク画像を生成します。

**検出色と HSV 範囲:**

| 色 | H (色相) | S (彩度) | V (明度) | 配置場所 |
|---|---|---|---|---|
| **青** | 100-125 | 100-255 | 30-255 | 右奥 (0.30, -0.30, 0.15) |
| **黄** | 20-35 | 100-255 | 60-255 | 左奥 (0.30, 0.30, 0.15) |
| **緑** | 40-80 | 100-255 | 30-255 | 前方 (0.40, 0.00, 0.15) |

### ノイズ除去処理

```cpp
// モルフォロジー変換でノイズ除去
cv::morphologyEx(img_thresholded, img_thresholded, cv::MORPH_OPEN,
    cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));
cv::morphologyEx(img_thresholded, img_thresholded, cv::MORPH_CLOSE,
    cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));

// エロージョンで隣接物体を分離
cv::morphologyEx(img_thresholded, img_thresholded, cv::MORPH_ERODE,
    cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
```

### 3D 位置推定

1. **輪郭検出**: `cv::findContours()` で物体の輪郭を抽出
2. **中心計算**: モーメントから物体の画像座標中心を算出
3. **深度取得**: 深度画像から 3x3 窓の中央値を取得（ノイズ低減）
4. **3D 座標**: カメラモデルで画像座標を 3D 光線に変換し、深度を乗算

```cpp
// カメラモデルで3D光線を計算
cv::Point3d ray = camera_model.projectPixelTo3dRay(rect_point);

// 深度情報と組み合わせて3D座標を取得
cv::Point3d camera_pos(
    ray.x * center_distance,
    ray.y * center_distance,
    ray.z * center_distance
);
```

## 動作フロー

### フェーズ 1: バッチスキャンと物体検出

1. カメラ開始姿勢（上方から見下ろす）に移動
2. 視野内の全ての色付き物体を検出
3. 作業エリア内の物体のみをターゲットリストに追加
4. 重複検出を除外（距離 5cm 以内）

**作業エリア定義:**
- X: 0.0 以上
- Y: -0.20 ～ 0.20

### フェーズ 2: 順次ピック＆プレース

各ターゲットに対して以下の動作を実行:

1. **接近**: ホバー高さ（0.20m）へ移動
2. **近距離再検出**: カメラで物体位置を再確認・補正（最大 3 回試行）
3. **下降**: 固定ピック高さ（0.095m）へ下降
4. **把持**: グリッパーを閉じる（5度）
5. **上昇**: ホバー高さまで上昇
6. **移動**: 色に対応した配置場所へ移動
7. **解放**: グリッパーを開く（60度）
8. **復帰**: カメラ開始姿勢に戻る

### 終了条件

スキャンで新しい物体が検出されなくなるまでフェーズ 1→2 を繰り返します。

## 主要なコード

### ColorDetector クラス

```cpp
class ColorDetector : public rclcpp::Node
{
public:
  explicit ColorDetector(const rclcpp::NodeOptions & options);

  // 最新の検出結果を取得
  DetectionResult getLatestDetection();
  std::vector<DetectionResult> getLatestDetections();

  // 検出結果をリセット
  void resetDetection();

private:
  // コールバック
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // TF2による座標変換
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
```

### MoveIt による軌道計画

```cpp
// MoveGroupの初期化
MoveGroupInterface move_group_arm(move_group_arm_node, "arm");
move_group_arm.setMaxVelocityScalingFactor(0.7);
move_group_arm.setMaxAccelerationScalingFactor(0.7);
move_group_arm.setPlanningTime(20.0);
move_group_arm.allowReplanning(true);

// デカルト空間での直線軌道
bool executeCartesianPath(
  MoveGroupInterface & move_group,
  const geometry_msgs::msg::Pose & target_pose,
  double max_step = 0.01)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(
    waypoints, max_step, 0.0, trajectory);

  if (fraction < 0.9) return false;

  move_group.execute(trajectory);
  return true;
}
```

### Gazebo での物体スポーン

```cpp
bool spawnObjectInGazebo(double x, double y, double z, Color color)
{
  // 色別のRGBA値を設定
  const std::map<Color, std::string> color_rgba = {
    {Color::BLUE,   "0.0 0.0 1.0 1"},
    {Color::YELLOW, "1.0 1.0 0.0 1"},
    {Color::GREEN,  "0.0 1.0 0.0 1"}
  };

  // ros_gz_simコマンドで動的に物体を生成
  snprintf(cmd, sizeof(cmd),
    "ros2 run ros_gz_sim create -world default -name '%s' "
    "-x %f -y %f -z %f -string '<sdf>...</sdf>'",
    name, x, y, z);

  return std::system(cmd) == 0;
}
```

## パラメータ設定

### カメラ関連

- **深度オフセット**: 0.015m（カメラ測定値の補正）
- **深度範囲**: 0.15m ～ 1.2m（検出有効範囲）
- **面積閾値**: 800 ピクセル（小さすぎる輪郭を除外）

### グリッパー

- **開**: 60 度
- **閉**: 5 度

### ピック＆プレース高さ

- **ホバー高さ**: 0.20m
- **ピック高さ**: 0.095m（固定）
- **配置高さ**: 0.15m

### MoveIt 設定

- **速度スケール**: 0.7（最大速度の 70%）
- **加速度スケール**: 0.7（最大加速度の 70%）
- **プランニング時間**: 20 秒
- **位置許容誤差**: 0.01m
- **姿勢許容誤差**: 0.05 rad

## トラブルシューティング

### 物体が検出されない場合

1. HSV 範囲が環境照明に合っているか確認
2. カメラの深度画像が正しく取得できているか確認（`ros2 topic echo /camera/aligned_depth_to_color/image_raw`）
3. TF フレームが正しく設定されているか確認（`ros2 run tf2_tools view_frames`）

### ピック位置がずれる場合

1. 深度オフセット（`DEPTH_OFFSET`）を調整
2. カメラキャリブレーションを確認
3. 近距離再検出の試行回数を増やす

### グリッパーで掴めない場合

1. ピック高さ（`PICK_Z`）を調整
2. グリッパー閉じ角度（`GRIPPER_CLOSE`）を調整
3. 物体のサイズと形状を確認

## 関連ドキュメント

- [開発ガイド](development.md) - 新しいプログラムの追加方法
- [実行方法](usage.md) - デモの実行手順
- [トラブルシューティング](troubleshooting.md) - よくある問題と解決方法
