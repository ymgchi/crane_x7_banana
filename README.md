# CRANE-X7 Banana

未来ロボティクス学科 ロボット設計制作論実習3

## 概要

このリポジトリは [NOPLAB/crane_x7_vla](https://github.com/NOPLAB/crane_x7_vla) をベースに、VLA/機械学習関連の機能を削除し、CRANE-X7 ロボットアームの基本制御に特化したものです。

- **元リポジトリ製作者**: NOPLAB  
- **ライセンス**: MIT License

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

### 2. Dockerイメージのビルド

```bash
./scripts/build.sh
```

### 3. 実行方法

#### 実機で実行する場合

```bash
# USB デバイスの権限設定
sudo chmod 666 /dev/ttyUSB0

# Docker Compose で自動起動（推奨）
docker compose --profile real up
```

または、開発用に手動で起動:

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

```bash
# X11 の権限設定
xhost +

# Docker Compose で起動
docker compose --profile sim up
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

### 別ターミナルでコンテナに接続

```bash
# 実行中のコンテナに接続
docker exec -it ros-dev /bin/bash

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
│   └── run.sh                 # 実行スクリプト
└── ros2/
    └── src/
        ├── crane_x7_description/   # ロボットモデル定義
        └── crane_x7_ros/           # 制御パッケージ
            ├── crane_x7_control/   # ハードウェア制御
            ├── crane_x7_examples/  # サンプルプログラム
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

### RT

- https://github.com/rt-net/crane_x7
- https://github.com/rt-net/crane_x7_ros
- https://github.com/rt-net/crane_x7_Hardware
- https://github.com/rt-net/crane_x7_samples

