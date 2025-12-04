# 色分別キューブ仕分けデモの実装

このドキュメントでは、2つの色分別デモ実装について解説します。

## デモ一覧

| デモ | ファイル | 言語 | 特徴 |
|------|----------|------|------|
| **color_sorting** | `src/color_sorting.cpp` | C++ | 単一ファイル、ビジュアルサーボイング |
| **point_cloud_sorting** | `scripts/point_cloud_sorting/` | Python + C++ | モジュール分離、RViz可視化 |

---

## 1. color_sorting（C++版）

### プログラム構成

```
crane_x7_examples/
├── src/color_sorting.cpp              # メインプログラム
├── launch/color_sorting.launch.py     # 起動ファイル
├── CMakeLists.txt                     # ビルド設定
└── package.xml                        # 依存関係定義
```

## 実装の特徴

### 1. 複数回検出による高精度位置合わせ

- ホバー位置で複数回（5回）検出して平均位置を計算
- 検出間隔200msで位置変動ノイズを低減
- ターゲット色を優先、検出できない場合は任意の色にフォールバック

### 2. ランダムキューブスポーン

- 5個のキューブを作業エリア内にランダム配置
- 各キューブの色（青/黄/緑）をランダムに決定
- 重複位置を避けた配置アルゴリズム

### 3. キューブ角度検出

- `cv::minAreaRect()` で最小外接矩形を計算
- キューブの回転角度を検出
- 把持姿勢をキューブの向きに合わせて最適化

### 4. ビジョンベースの物体検出

- **OpenCV** による HSV 色空間での色検出
- **RealSense D435** カメラによる RGB-D センシング
- 深度情報を使った 3D 位置推定
- 中央値フィルタによる深度ノイズ除去

### 5. TF2 による座標変換

- カメラ座標系 → ロボットベース座標系の変換
- 複数フレーム候補への自動フォールバック
- 名前空間/プレフィックスの違いに対応

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

// エロージョンで隣接キューブを分離
cv::morphologyEx(img_thresholded, img_thresholded, cv::MORPH_ERODE,
    cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
```

### 3D 位置推定

1. **輪郭検出**: `cv::findContours()` でキューブの輪郭を抽出
2. **角度検出**: `cv::minAreaRect()` で回転角度を算出
3. **中心計算**: モーメントからキューブの画像座標中心を算出
4. **深度取得**: 深度画像から 5x5 窓の中央値を取得（ノイズ低減を強化）
5. **3D 座標**: カメラモデルで画像座標を 3D 光線に変換し、深度を乗算

## 動作フロー

### フェーズ 0: キューブスポーン

1. 作業エリア内に5個のキューブをランダム配置
2. 各キューブに青/黄/緑のいずれかの色をランダム割り当て
3. 重複位置を避けて配置

### フェーズ 1: バッチスキャンと物体検出

1. カメラ開始姿勢（上方から見下ろす）に移動
2. 視野内の全ての色付きキューブを検出
3. 作業エリア内のキューブのみをターゲットリストに追加
4. 重複検出を除外（距離 4cm 以内）

**作業エリア定義:**
- X: 0.00 ～ 0.50
- Y: -0.40 ～ 0.40
- Z: -0.20 より上（TF変換エラー検出のため）
- ロボットからの距離: 0.50m以内

### フェーズ 2: 順次ピック＆プレース

各ターゲットに対して以下の動作を実行:

1. **接近**: ホバー高さ（0.20m）へ移動
2. **再検出**: ホバー位置で複数回検出（5回）して平均位置を取得
   - 検出間隔: 200ms
   - ターゲット色を優先、検出できない場合は任意の色にフォールバック
3. **角度調整**: 検出した回転角度に合わせてグリッパー姿勢を調整
4. **下降**: 動的ピック高さへ下降
   - Z座標が信頼できる場合: 検出Z - 0.005m
   - Z座標が低すぎる場合: 固定高さ 0.095m（テーブル面誤検出対策）
5. **把持**: グリッパーを閉じる（20度）
6. **上昇**: 運搬高さ（0.30m）まで持ち上げ（配置済みキューブとの衝突回避）
7. **移動**: 色に対応した配置場所へ移動
8. **解放**: グリッパーを開く（60度）
9. **復帰**: カメラ開始姿勢に戻る

### 終了条件

スキャンで新しいキューブが検出されなくなるまでフェーズ 1→2 を繰り返します。

## 主要なコード

### 複数回検出による平均位置取得

```cpp
// 複数回検出して平均位置を返す（ノイズ低減）
DetectionResult getAveragedDetection(
    int num_samples = 5,
    int wait_ms = 200,
    Color target_color = Color::NONE,
    bool allow_fallback = true)
{
    std::vector<DetectionResult> samples;
    Color detected_color = Color::NONE;

    for (int i = 0; i < num_samples; i++) {
        // 複数色検出を試みる
        auto results = detectAllColors();

        DetectionResult best;
        // ターゲット色を優先、なければフォールバック
        if (target_color != Color::NONE) {
            for (const auto& r : results) {
                if (r.color == target_color) {
                    best = r;
                    break;
                }
            }
        }
        if (!best.valid && allow_fallback && !results.empty()) {
            best = results[0];  // 任意の検出結果を使用
        }

        if (best.valid) {
            samples.push_back(best);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    }

    // 平均位置を計算
    DetectionResult averaged = samples[0];
    averaged.pose.position.x = 0.0;
    averaged.pose.position.y = 0.0;
    averaged.pose.position.z = 0.0;

    for (const auto& sample : samples) {
        averaged.pose.position.x += sample.pose.position.x;
        averaged.pose.position.y += sample.pose.position.y;
        averaged.pose.position.z += sample.pose.position.z;
    }
    averaged.pose.position.x /= samples.size();
    averaged.pose.position.y /= samples.size();
    averaged.pose.position.z /= samples.size();

    return averaged;
}
```

### 角度検出

```cpp
double detectCubeAngle(const cv::Mat& mask) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return 0.0;

    // 最大面積の輪郭を使用
    auto max_contour = *std::max_element(contours.begin(), contours.end(),
        [](const auto& a, const auto& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });

    // 最小外接矩形から角度を取得
    cv::RotatedRect rect = cv::minAreaRect(max_contour);
    return rect.angle;
}
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

### Gazebo でのランダムスポーン

```cpp
void spawnRandomCubes(int num_cubes = 5) {
    std::vector<Color> colors = {Color::BLUE, Color::YELLOW, Color::GREEN};
    std::vector<std::pair<double, double>> positions;

    for (int i = 0; i < num_cubes; i++) {
        // ランダムな色を選択
        Color color = colors[rand() % colors.size()];

        // 重複しない位置を生成
        double x, y;
        do {
            x = 0.15 + (rand() / (double)RAND_MAX) * 0.25;  // 0.15-0.40
            y = -0.20 + (rand() / (double)RAND_MAX) * 0.40; // -0.20-0.20
        } while (isPositionOccupied(positions, x, y, 0.06));

        positions.push_back({x, y});
        spawnCube(x, y, TABLE_HEIGHT + 0.02, color);
    }
}
```

## パラメータ設定

### 再検出（平均位置取得）

- **検出回数**: 5回
- **検出間隔**: 200ms
- **フォールバック**: ターゲット色が検出できない場合は任意の色を使用

### カメラ関連

- **深度オフセット**: 0.015m（カメラ測定値の補正）
- **深度範囲**: 0.15m ～ 1.2m（検出有効範囲）
- **深度フィルタ窓**: 5x5 ピクセル（中央値フィルタでノイズ除去強化）
- **面積閾値**: 1000 ピクセル（ノイズを除外）

### グリッパー

- **開**: 60 度
- **閉**: 20 度（5cmキューブ用）

### ピック＆プレース高さ

- **ホバー高さ**: 0.20m
- **ピック高さ**: 動的調整（条件付き）
  - 検出Z座標 > 0.05m の場合: 検出Z - 0.005m（動的調整）
  - 検出Z座標 ≤ 0.05m の場合: 0.095m（固定高さ、テーブル面誤検出対策）
- **運搬時持ち上げ高さ**: 0.30m（配置済みキューブとの衝突回避）
- **配置高さ**: 0.15m

### MoveIt 設定

- **速度スケール**: 0.7（最大速度の 70%）
- **加速度スケール**: 0.7（最大加速度の 70%）
- **プランニング時間**: 20 秒
- **位置許容誤差**: 0.01m
- **姿勢許容誤差**: 0.05 rad

## トラブルシューティング

### キューブが検出されない場合

1. HSV 範囲が環境照明に合っているか確認
2. カメラの深度画像が正しく取得できているか確認（`ros2 topic echo /camera/aligned_depth_to_color/image_raw`）
3. TF フレームが正しく設定されているか確認（`ros2 run tf2_tools view_frames`）

### ピック位置がずれる場合

1. ビジュアルサーボイングのログを確認（収束しているか）
2. グリッパー・カメラオフセットの値を調整
3. 深度オフセット（`DEPTH_OFFSET`）を調整
4. カメラキャリブレーションを確認

### グリッパーで掴めない場合

1. ログで検出Z座標を確認
   - Z座標が負またはテーブル面以下 → カメラ・深度センサーのキャリブレーション確認
   - 固定高さが使われている → `MIN_VALID_Z`（0.05m）や`FIXED_PICK_Z`（0.095m）を調整
2. ピック高さオフセット（`PICK_Z_OFFSET`）を調整（デフォルト: 0.005m）
3. グリッパー閉じ角度（`GRIPPER_CLOSE`）を調整
4. キューブのサイズと形状を確認

### 運搬時に配置済みキューブにぶつかる場合

1. 運搬時持ち上げ高さ（`PICK_Z_LIFT`）を増やす（デフォルト: 0.30m）
2. 配置場所の位置を調整して経路を変更

---

## 2. point_cloud_sorting（Python版）

### プログラム構成

```
crane_x7_examples/
├── scripts/
│   ├── point_cloud_sorting_node.py       # メインノード
│   └── point_cloud_sorting/              # Pythonモジュール
│       ├── __init__.py                   # パッケージ初期化
│       ├── color_detector.py             # HSV色検出
│       ├── edge_grasp_finder.py          # エッジ検出（点群ベース）
│       └── robot_controller.py           # ロボット制御（サービスクライアント）
├── src/
│   └── motion_service_node.cpp           # MoveItサービスノード（C++）
└── launch/
    └── point_cloud_sorting.launch.py     # 起動ファイル
```

### アーキテクチャ

```
┌─────────────────────────────────────────────────────────────┐
│                    Python Node                               │
│  ┌─────────────────┐  ┌─────────────────┐  ┌──────────────┐ │
│  │  ColorDetector  │  │ EdgeGraspFinder │  │RobotController│ │
│  │  (HSV検出)      │  │ (点群エッジ)    │  │(サービス呼出) │ │
│  └─────────────────┘  └─────────────────┘  └──────────────┘ │
└───────────────────────────────┬─────────────────────────────┘
                                │ ROS 2 Services
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                  C++ Motion Service Node                     │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │              MoveGroupInterface (MoveIt)                 │ │
│  │  - /motion/move_to_camera_pose                          │ │
│  │  - /motion/open_gripper                                 │ │
│  │  - /motion/close_gripper                                │ │
│  │  - /motion/execute_pose                                 │ │
│  │  - /motion/execute_cartesian                            │ │
│  └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### 実装の特徴

#### 1. C++/Python分離アーキテクチャ

- **C++ (motion_service_node.cpp)**: MoveIt MoveGroupInterfaceをラップしたROS 2サービス
- **Python (point_cloud_sorting_node.py)**: メインロジック、サービスクライアントとして動作

この分離により:
- Pythonでの高レベルロジック記述が容易
- MoveItの複雑な初期化をC++側で管理
- デバッグ・開発が高速化

#### 2. モジュール化された検出システム

- **ColorDetector**: HSV色空間での色検出（color_sorting.cppからポート）
- **EdgeGraspFinder**: 点群のエッジ検出による把持点精密化
- **RobotController**: サービスベースのロボット制御

#### 3. RViz可視化

以下のトピックでRVizに検出結果を表示:
- `/edge_detection/cloud_filtered` - フィルタリング後の点群
- `/edge_detection/edges` - 検出されたエッジポイント
- `/edge_detection/grasp_marker` - 把持候補点マーカー

### パラメータ設定

#### メインノード (point_cloud_sorting_node.py)

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `num_objects` | 5 | スポーンするキューブの数 |
| `use_edge_detection` | true | エッジ検出を使用するか |
| `max_iterations` | 10 | 最大イテレーション回数 |

#### ロボット制御 (robot_controller.py)

| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `pick_z_above` | 0.20m | ホバー高さ |
| `pick_z_lift` | 0.30m | 持ち上げ高さ |
| `stabilize_wait` | 0.85秒 | 把持前待機時間 |

### 動作フロー

1. **初期化**: カメラデータ・サービス待機
2. **スポーン**: 5個のキューブをランダム配置
3. **スキャン**: カメラ姿勢で全キューブ検出
4. **ピック＆プレース** (最大10イテレーション):
   - ホバー位置へ移動
   - グリッパーを開く
   - ピック位置へ降下
   - **0.85秒待機** (アーム安定化)
   - グリッパーを閉じる
   - 持ち上げ
   - 配置場所へ移動
   - グリッパーを開く
5. **完了**: 全キューブ処理完了

### ROS 2 サービス一覧

| サービス名 | 型 | 説明 |
|-----------|-----|------|
| `/motion/move_to_camera_pose` | std_srvs/Trigger | カメラ観察姿勢へ移動 |
| `/motion/open_gripper` | std_srvs/Trigger | グリッパーを開く |
| `/motion/close_gripper` | std_srvs/Trigger | グリッパーを閉じる |
| `/motion/execute_pose` | std_srvs/Trigger | 目標姿勢へ移動 |
| `/motion/execute_cartesian` | std_srvs/Trigger | デカルト空間で移動 |

目標姿勢は `/motion/target_pose` トピック (geometry_msgs/PoseStamped) で事前に設定します。

---

## 関連ドキュメント

- [開発ガイド](development.md) - 新しいプログラムの追加方法
- [実行方法](usage.md) - デモの実行手順
- [トラブルシューティング](troubleshooting.md) - よくある問題と解決方法
