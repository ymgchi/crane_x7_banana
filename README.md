# CRANE-X7 Banana

未来ロボティクス学科 ロボット設計制作論実習3

## 概要

このリポジトリは [NOPLAB/crane_x7_vla](https://github.com/NOPLAB/crane_x7_vla) をベースに、VLA/機械学習関連の機能を削除し、CRANE-X7 ロボットアームの基本制御に特化したものです。

**開発者**: ymgchi  Nekomaru　TomiKazu-git
**所属**: 未来ロボティクス学科  
**用途**: ロボット設計制作論実習3  
**ライセンス**: MIT License (詳細は下部の「ライセンスと著作権」セクションを参照)

### 使用しているサブモジュール

このリポジトリは以下の外部パッケージをGitサブモジュールとして使用しています:

- **crane_x7_ros**: [NOPLAB/crane_x7_ros](https://github.com/NOPLAB/crane_x7_ros) (Apache License 2.0)
  - RT-net公式版ではGazeboシミュレーターでエラーが発生するため、NOPLABのフォーク版を使用
- **crane_x7_description**: [rt-net/crane_x7_description](https://github.com/rt-net/crane_x7_description) (NON-COMMERCIAL LICENSE)

## 必要環境

- **Native Linux** (Ubuntu 22.04 推奨)
- **Docker** & **Docker Compose**
- **CRANE-X7 ロボットアーム** (実機使用時)

## クイックスタート

### 1. リポジトリのクローン

```bash
git clone --recursive https://github.com/ymgchi/crane_x7_banana.git
cd crane_x7_banana
```

### 2. 環境変数ファイルの作成

```bash
# .env.template をコピーして .env を作成
cp .env.template .env

# 必要に応じて編集（通常はそのままでOK）
# USB_DEVICE=/dev/ttyUSB0  # 実機のUSBデバイスパス
# DISPLAY=:0               # シミュレーター用のディスプレイ設定
```

### 3. 実行方法

#### 実機で実行する場合

**簡単な方法（推奨）:**

```bash
# USB デバイスの権限設定
sudo chmod 666 /dev/ttyUSB0

# ビルド＆起動を一発で実行
./scripts/run_real.sh
```

**または Docker Compose を直接使用:**

```bash
# USB デバイスの権限設定
sudo chmod 666 /dev/ttyUSB0

# ビルドして起動
docker compose --profile real up --build
```

**開発用に手動で起動:**

```bash
# コンテナに入る
./scripts/run.sh

# コンテナ内で実行
cd /workspace/ros2
colcon build --symlink-install
source install/setup.bash
ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0
```

#### シミュレーター（Gazebo）で実行する場合

**簡単な方法（推奨）:**

```bash
# ビルド＆起動を一発で実行
./scripts/run_sim.sh
```

**または Docker Compose を直接使用:**

```bash
# X11 の権限設定
xhost +

# ビルドして起動
docker compose --profile sim up --build
```

## 開発ガイド

### 開発環境のセットアップ

```bash
# 1. X11 の権限設定
xhost +

# 2. Docker イメージをビルド
./scripts/build.sh

# 3. 開発用コンテナを起動
./scripts/run.sh
```

### コンテナ内での作業

```bash
# ワークスペースに移動
cd /workspace/ros2

# パッケージをビルド
colcon build --symlink-install

# 環境変数を読み込む
source install/setup.bash

# デモを起動（実機）
ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0

# または、特定のサンプルを実行
ros2 launch crane_x7_examples example.launch.py example:='gripper_control'
```

### 利用可能なサンプル

`example.launch.py` で実行できるサンプル一覧:

- `gripper_control` - グリッパーの開閉制御
- `pose_groupstate` - 事前定義されたポーズへの移動
- `joint_values` - 関節角度を指定した制御
- `pick_and_place` - ピック＆プレース動作
- `cartesian_path` - デカルト空間での軌道追従

### バナナ仕分けデモ

#### 概要

バナナ仕分けデモは、CRANE-X7ロボットアームを使用して物体を固定位置から3箇所に自動仕分けするプログラムです。MoveItを使用した軌道計画とグリッパー制御を組み合わせることで、ピック＆プレース動作を実現しています。

**特徴:**
- 固定位置の物体を3箇所（右・中央・左）に順次配置
- デカルト空間での直線軌道生成（Cartesian Path Planning）
- 安全性を考慮した速度・加速度制限（30%）
- Gazeboシミュレーターでの物体位置自動リセット機能

#### ファイル構成

- **実行ファイル**: `ros2/src/crane_x7_ros/crane_x7_examples/src/banana.cpp`
  - C++で実装されたメインプログラム
  - MoveItの`MoveGroupInterface`を使用してアームとグリッパーを制御
  - 328行のコードで包括的な制御フローを実装

- **起動ファイル**: `ros2/src/crane_x7_ros/crane_x7_examples/launch/banana.launch.py`
  - ROS2のLaunchファイル
  - ロボットの記述情報とMoveIt設定を読み込み
  - `use_sim_time`パラメータでシミュレーション/実機の切り替えが可能

#### 実行方法

##### シミュレーター（Gazebo）で実行

**方法1: 別ターミナルから実行（推奨）**

```bash
# ターミナル1: Gazeboシミュレーターを起動
./scripts/run_sim.sh

# ターミナル2: バナナ仕分けデモを実行
docker exec -it ros-dev-banana /bin/bash
source /workspace/ros2/install/setup.bash
ros2 launch crane_x7_examples banana.launch.py
```

**方法2: コンテナ内で実行**

```bash
# コンテナに入る
./scripts/run.sh

# ビルド
cd /workspace/ros2
colcon build --symlink-install
source install/setup.bash

# Gazeboを起動（別ターミナルまたはバックグラウンドで）
ros2 launch crane_x7_gazebo crane_x7_with_table.launch.py

# バナナデモを実行
ros2 launch crane_x7_examples banana.launch.py use_sim_time:=true
```

##### 実機で実行（オプション）

```bash
# 実機を接続してコンテナに入る
./scripts/run_real.sh

# コンテナ内で
cd /workspace/ros2
source install/setup.bash
ros2 launch crane_x7_examples banana.launch.py use_sim_time:=false port_name:=/dev/ttyUSB0
```

**注意**: 実機で実行する場合は、周囲の安全を確認し、物体の位置を適切に配置してください。

#### 動作フロー

各タスク（3回繰り返し）で以下のステップを実行します:

1. **グリッパーを開く** - 物体を掴む準備
2. **ピック上方位置へ移動** - 物体の上に移動
3. **下降してピック位置へ** - 直線軌道で下降
4. **グリッパーを閉じる** - 物体を掴む
5. **上昇** - 物体を持ち上げる
6. **配置上方位置へ移動** - 目標位置の上に移動
7. **下降して配置位置へ** - 直線軌道で下降
8. **グリッパーを開く** - 物体を離す
9. **上昇** - 配置位置から離れる
10. **ホーム位置へ戻る** - 初期位置に戻る
11. **物体位置リセット** - 次のタスクのため物体を元の位置に戻す（Gazeboのみ）

#### カスタマイズ

`banana.cpp`内の座標を変更することで、ピック＆プレース位置を調整できます:

```cpp
// 各位置の定義（X, Y, Z, Roll, Pitch, Yaw）
auto pick_pose = createPose(0.2, 0.0, 0.13, -180, 0, -90);           // ピック位置
auto place_pose_1 = createPose(0.2, 0.15, 0.13, -180, 0, -90);      // 配置位置1（右）
auto place_pose_2 = createPose(0.2, 0.0, 0.13, -180, 0, -90);       // 配置位置2（中央）
auto place_pose_3 = createPose(0.2, -0.15, 0.13, -180, 0, -90);     // 配置位置3（左）
```

また、速度やタイミングも調整可能です:

```cpp
move_group_arm.setMaxVelocityScalingFactor(0.3);      // 速度スケール（0.1-1.0）
move_group_arm.setMaxAccelerationScalingFactor(0.3);  // 加速度スケール（0.1-1.0）
```

#### トラブルシューティング

**プログラムが起動しない場合:**

```bash
# パッケージを再ビルド
cd /workspace/ros2
colcon build --packages-select crane_x7_examples --symlink-install
source install/setup.bash
```

**動作が不安定な場合:**
- 速度・加速度のスケーリングファクターを下げる（現在30%）
- 物体の位置や重さを確認
- MoveItの計画失敗メッセージを確認

**Gazeboで物体がリセットされない場合:**
- Gazeboのサービスが起動しているか確認: `ros2 service list | grep set_entity_state`
- 物体の名前が`target_object`と一致しているか確認

### 別ターミナルでコンテナに接続

```bash
# 実行中のコンテナに接続
docker exec -it ros-dev-banana /bin/bash

# 環境変数を読み込む
source /workspace/ros2/install/setup.bash

# コマンドを実行
ros2 topic list
```

## プロジェクト構造

```
crane_x7_banana/
├── docker-compose.yml          # Docker Compose 設定
├── Dockerfile                  # Docker イメージ定義
├── .env                        # 環境変数設定
├── scripts/
│   ├── build.sh               # ビルドスクリプト
│   ├── run.sh                 # 実行スクリプト
│   ├── run_real.sh            # 実機用起動スクリプト
│   └── run_sim.sh             # シミュレーター用起動スクリプト
└── ros2/
    └── src/
        ├── crane_x7_description/   # ロボットモデル定義
        └── crane_x7_ros/           # 制御パッケージ
            ├── crane_x7/
            ├── crane_x7_control/   # ハードウェア制御
            ├── crane_x7_examples/  # サンプルプログラム（banana.cppを含む）
            ├── crane_x7_gazebo/    # シミュレーター設定
            └── crane_x7_moveit_config/  # MoveIt設定
```

## 環境変数設定

`.env` ファイルで以下の変数を設定できます:

```bash
# 実機用: USB デバイスパス
USB_DEVICE=/dev/ttyUSB0

# シミュレーター用: ディスプレイ設定
DISPLAY=:0
```

## トラブルシューティング

### USB デバイスが見つからない

```bash
# デバイスを確認
ls /dev/ttyUSB*

# 権限を設定
sudo chmod 666 /dev/ttyUSB0
```

### パッケージが見つからない

```bash
# ワークスペースを再ビルド
cd /workspace/ros2
colcon build --symlink-install
source install/setup.bash
```

### X11 表示エラー

```bash
# ホスト側で実行
xhost +
```

## 参考情報

### CRANE-X7 関連

- https://github.com/rt-net/crane_x7
- https://github.com/rt-net/crane_x7_ros
- https://github.com/rt-net/crane_x7_Hardware
- https://github.com/rt-net/crane_x7_samples

## ライセンスと著作権

### このリポジトリ

- **著作権**: Copyright (c) 2025 ymgchi
- **ライセンス**: MIT License
- **文責**: ymgchi

### 元リポジトリ

このプロジェクトは [NOPLAB/crane_x7_vla](https://github.com/NOPLAB/crane_x7_vla) をベースにしています。

- **元リポジトリ著作権**: Copyright (c) 2025 nop (NOPLAB)
- **元リポジトリライセンス**: MIT License

### CRANE-X7 ハードウェア・ソフトウェア

CRANE-X7 ロボットアームおよび関連するROS2パッケージは株式会社アールティが開発・提供しています。

- **CRANE-X7 関連パッケージ著作権**: Copyright 2022 RT Corporation
- **ライセンス**: Apache License 2.0

詳細は各パッケージの LICENSE ファイルを参照してください。

