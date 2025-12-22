# 実行方法

## シミュレーターで実行

### 1. シミュレーター環境を起動

```bash
xhost +
docker compose --profile sim up
```

### 2. 色分別キューブ仕分けデモを実行

別のターミナルで:

```bash
docker exec -it ros-dev-banana /bin/bash
source /opt/ros/humble/setup.bash && source /workspace/ros2/install/setup.bash
ros2 launch crane_x7_examples color_sorting.launch.py use_sim_time:=true
```

デモが開始すると:
1. 5個のキューブがランダムな色で自動スポーン
2. カメラで全キューブをスキャン（RVizで可視化）
3. 各キューブの位置を検出して把持
4. 色ごとに指定の場所へ配置

## 実機で実行

### 必要な機材

- CRANE-X7 ロボットアーム
- RealSense D435 カメラ（アームに取り付け済み）
- 5cm角の色付きキューブ（オレンジ・黄・緑）

### 1. キューブを配置

デモ開始前に、テーブル上の作業エリア内にキューブを配置してください。

**作業エリア（ロボット座標系 base_link）:**

```
        ← Y+ (左)
         │
    ┌────┼────┐
    │    │    │  X: 0.10 〜 0.40 m
    │ 作業エリア │  Y: -0.15 〜 0.25 m
    │    │    │
    └────┼────┘
         │
      [ROBOT] → X+ (前方)
```

**推奨配置位置（5個の場合）:**

| 位置 | X (m) | Y (m) | 備考 |
|------|-------|-------|------|
| 1 | 0.22 | -0.12 | 右手前 |
| 2 | 0.30 | -0.06 | 右奥 |
| 3 | 0.22 | 0.12 | 左手前 |
| 4 | 0.30 | 0.06 | 左奥 |
| 5 | 0.26 | 0.00 | 中央 |

### 2. USB権限を設定してコンテナを起動

**ターミナル1:**

```bash
# ロボットアームのUSBポートに権限を付与
sudo chmod 666 /dev/ttyUSB0

# X11表示を許可
xhost +

# コンテナを起動（RealSenseカメラ + ロボットアーム）
docker compose --profile real up
```

起動すると以下が自動で開始されます:
- ロボットアームの制御ノード
- RealSenseカメラノード（RGB + 深度 + 点群）
- MoveIt モーションプランナー
- RViz 可視化ツール

### 3. 色分別デモを実行

**ターミナル2:**

```bash
# コンテナに接続
docker exec -it ros-dev-banana /bin/bash

# 環境変数を読み込み
source /opt/ros/humble/setup.bash && source /workspace/ros2/install/setup.bash

# デモを実行
ros2 launch crane_x7_examples color_sorting.launch.py use_sim_time:=false
```

### 4. デモの動作

1. **カメラ観測位置へ移動** - アームが上から見下ろす位置へ
2. **スポーン試行** - Gazebo用コマンドが失敗（無視してOK）
3. **スキャン開始** - カメラでキューブを検出
4. **ピック＆プレース** - 検出した各キューブを色別に仕分け

**仕分け先:**
- オレンジ色キューブ → 右奥 (0.30, -0.30)
- 黄色キューブ → 左奥 (0.30, 0.30)
- 緑色キューブ → 前方 (0.45, 0.00)

### 5. 終了方法

```bash
# ターミナル2: Ctrl+C でデモを停止

# ターミナル1: Ctrl+C でコンテナを停止
# または
docker stop ros-dev-banana
```

## トラブルシューティング（実機）

### カメラが認識されない

```bash
# RealSenseが接続されているか確認
lsusb | grep Intel

# カメラトピックが配信されているか確認
docker exec ros-dev-banana ros2 topic list | grep camera
```

### ロボットが動かない

```bash
# USB権限を再設定
sudo chmod 666 /dev/ttyUSB0

# joint_statesトピックを確認
docker exec ros-dev-banana ros2 topic echo /joint_states --once
```

### キューブが検出されない

- 照明条件を確認（明るすぎ/暗すぎ）
- キューブが作業エリア内にあるか確認
- HSV色範囲の調整が必要な場合あり

## その他のサンプルプログラム

`example.launch.py` で実行できるサンプル一覧:

| サンプル名        | 説明                         | 実行コマンド                                                                 |
| ----------------- | ---------------------------- | ---------------------------------------------------------------------------- |
| `gripper_control` | グリッパーの開閉制御         | `ros2 launch crane_x7_examples example.launch.py example:='gripper_control'` |
| `pose_groupstate` | 事前定義されたポーズへの移動 | `ros2 launch crane_x7_examples example.launch.py example:='pose_groupstate'` |
| `joint_values`    | 関節角度を指定した制御       | `ros2 launch crane_x7_examples example.launch.py example:='joint_values'`    |
| `pick_and_place`  | ピック＆プレース動作         | `ros2 launch crane_x7_examples example.launch.py example:='pick_and_place'`  |
| `cartesian_path`  | デカルト空間での軌道追従     | `ros2 launch crane_x7_examples example.launch.py example:='cartesian_path'`  |

**Gazebo で実行する場合**: `use_sim_time:='true'` を追加

```bash
ros2 launch crane_x7_examples example.launch.py example:='gripper_control' use_sim_time:='true'
```

## 関連ドキュメント

- [セットアップ](setup.md) - 初期設定
- [色分別デモ実装詳細](color-sorting-implementation.md) - アルゴリズムの解説
- [トラブルシューティング](troubleshooting.md) - 問題が発生した場合
