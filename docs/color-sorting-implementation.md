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

- **動的ピック高さ調整（フォールバック付き）**: 検出したZ座標に基づいて下降高さを自動計算、不正な値の場合は固定高さを使用
- **近距離再検出**: 3回測定の平均化で位置補正（色フィルタリング付き）
- **5x5深度フィルタ**: より多くのピクセルから中央値を取得してノイズ除去
- **運搬時高さ確保**: 配置済みキューブとの衝突を回避
- **デカルト空間での直線軌道計画**: 滑らかな動作
- **MoveIt の再プランニング機能の活用**: 失敗時の自動リトライ

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
3. **深度取得**: 深度画像から 5x5 窓の中央値を取得（ノイズ低減を強化）
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
- X: 0.00 ～ 0.50
- Y: -0.40 ～ 0.40
- Z: -0.20 より上（TF変換エラー検出のため）
- ロボットからの距離: 0.50m以内

### フェーズ 2: 順次ピック＆プレース

各ターゲットに対して以下の動作を実行:

1. **接近**: ホバー高さ（0.20m）へ移動
2. **近距離再検出**: カメラで物体位置を5回測定して平均化・補正
   - ターゲット色を優先的に検索
   - ターゲット色が見つからない場合は他の色でも位置精度のため受け入れ
   - 200ms間隔で測定
3. **下降**: 動的ピック高さへ下降
   - Z座標が信頼できる場合: 検出Z - 0.005m
   - Z座標が低すぎる場合: 固定高さ 0.095m（テーブル面誤検出対策）
4. **把持**: グリッパーを閉じる（5度）
5. **上昇**: 運搬高さ（0.30m）まで持ち上げ（配置済みキューブとの衝突回避）
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
- **深度フィルタ窓**: 5x5 ピクセル（中央値フィルタでノイズ除去強化）
- **面積閾値**: 1000 ピクセル（ノイズを除外、800→1000に変更）

### HSV 色検出範囲（ROSパラメータで調整可能）

実行時に `--ros-args -p` で各色のHSV範囲をカスタマイズできます：

```bash
ros2 launch crane_x7_examples color_sorting.launch.py \
  --ros-args \
  -p blue.h_min:=100 -p blue.h_max:=125 \
  -p yellow.h_min:=20 -p yellow.h_max:=35 \
  -p green.h_min:=40 -p green.h_max:=80
```

### グリッパー

- **開**: 60 度
- **閉**: 5 度

### ピック＆プレース高さ

- **ホバー高さ**: 0.20m
- **ピック高さ**: 動的調整（条件付き）
  - 検出Z座標 > 0.05m の場合: 検出Z - 0.005m（動的調整）
  - 検出Z座標 ≤ 0.05m の場合: 0.095m（固定高さ、テーブル面誤検出対策）
- **運搬時持ち上げ高さ**: 0.30m（配置済みキューブとの衝突回避）
- **配置高さ**: 0.15m

### 精度向上機能

- **近距離再検出**: 5回測定の平均化（ターゲット色優先、200ms間隔）
- **位置フィルタリング**: 作業エリア範囲チェック、距離チェック（0.50m以内）
- **重複検出閾値**: 0.04m（4cm、統一値）

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
3. 近距離再検出の測定回数やフィルタリング条件を調整

### グリッパーで掴めない場合

1. ログで検出Z座標を確認
   - Z座標が負またはテーブル面以下 → カメラ・深度センサーのキャリブレーション確認
   - 固定高さが使われている → `MIN_VALID_Z`（0.05m）や`FIXED_PICK_Z`（0.095m）を調整
2. ピック高さオフセット（`PICK_Z_OFFSET`）を調整（デフォルト: 0.005m）
3. グリッパー閉じ角度（`GRIPPER_CLOSE`）を調整
4. 物体のサイズと形状を確認

### 運搬時に配置済みキューブにぶつかる場合

1. 運搬時持ち上げ高さ（`PICK_Z_LIFT`）を増やす（デフォルト: 0.30m）
2. 配置場所の位置を調整して経路を変更

## 関連ドキュメント

- [開発ガイド](development.md) - 新しいプログラムの追加方法
- [実行方法](usage.md) - デモの実行手順
- [トラブルシューティング](troubleshooting.md) - よくある問題と解決方法
