# CRANE-X7 Banana

**未来ロボティクス学科 ロボット設計制作論実習 3**

CRANE-X7 ロボットアームを使用したバナナ仕分けデモンストレーション

## 概要

物体を固定位置から 3 箇所（右・中央・左）に順次配置する動作を自動で実行します。

**開発者**: ymgchi Nekomaru TomiKazu-git

### 使用技術

- ROS 2 Humble
- MoveIt (モーションプランニング)
- Gazebo (物理シミュレーション)
- Docker (環境構築)

### 動作環境

- Ubuntu 22.04
- Docker
- NVIDIA GPU (推奨)

## クイックスタート

### 1. セットアップ

```bash
git clone --recursive https://github.com/ymgchi/crane_x7_banana.git
cd crane_x7_banana
cp .env.template .env
```

### 2. シミュレーターで実行

```bash
xhost +
./scripts/run_sim.sh
```

別ターミナルで:

```bash
docker exec -it ros-dev-banana /bin/bash
source /opt/ros/humble/setup.bash
cd /workspace/ros2
source install/setup.bash
ros2 launch crane_x7_examples banana.launch.py use_sim_time:=true
```

### 3. 実機で実行

```bash
sudo chmod 666 /dev/ttyUSB0
./scripts/run_real.sh
```

別ターミナルで:

```bash
docker exec -it ros-dev-banana /bin/bash
source /opt/ros/humble/setup.bash
cd /workspace/ros2
source install/setup.bash
ros2 launch crane_x7_examples banana.launch.py use_sim_time:=false
```

## ドキュメント

| ドキュメント | 説明 |
| --- | --- |
| [セットアップ](docs/setup.md) | 環境構築の詳細手順 |
| [実行方法](docs/usage.md) | シミュレーター/実機での実行方法 |
| [実装詳細](docs/implementation.md) | バナナ仕分けデモの実装解説 |
| [開発ガイド](docs/development.md) | 開発環境と新規プログラム追加 |
| [トラブルシューティング](docs/troubleshooting.md) | よくある問題と解決方法 |
| [ライセンス](docs/license.md) | ライセンスと著作権情報 |
| [参考情報](docs/references.md) | 関連リンク集 |

## ライセンス

- **このリポジトリ**: MIT License (Copyright (c) 2025 ymgchi)
- **crane_x7_ros**: Apache License 2.0
- **crane_x7_description**: NON-COMMERCIAL LICENSE

詳細は [docs/license.md](docs/license.md) を参照してください。

---

**Last Updated**: 2025 年 11 月 19 日
