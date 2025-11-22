# CRANE-X7 Banana

**未来ロボティクス学科 ロボット設計制作論実習 3**

CRANE-X7 ロボットアームを使用したデモンストレーション

## 概要

このリポジトリには以下の2つのデモプログラムが含まれています：

### 1. 色分別バナナ仕分けデモ（color_sorting）
カメラで青・黄・緑の3色のバナナを検出し、色ごとに異なる場所へ自動仕分けします。
- **青色**: 右奥に配置
- **黄色**: 左奥に配置
- **緑色**: 前方に配置

### 2. バナナ仕分けデモ（banana）
物体を固定位置から 3 箇所（右・中央・左）に順次配置する動作を自動で実行します。

## デモ

https://github.com/user-attachments/assets/4839138e-3cad-45b9-bf47-933769d0ca6d

### 使用技術

- ROS 2 Humble
- MoveIt (モーションプランニング)
- Gazebo (物理シミュレーション)
- OpenCV (色検出・画像処理)
- RealSense カメラ (深度センシング)
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
# X11の権限設定（GUIアプリケーション表示に必要）
xhost +

# シミュレーター環境でコンテナを起動
docker compose --profile sim up --build
```

別ターミナルで以下を実行:

```bash
# コンテナに接続
docker exec -it ros-dev-banana /bin/bash

# ROS 2環境をセットアップ
source /opt/ros/humble/setup.bash
cd /workspace/ros2
source install/setup.bash

# 色分別バナナ仕分けデモを実行する場合
ros2 launch crane_x7_examples color_sorting.launch.py use_sim_time:=true

# または、バナナ仕分けデモを実行する場合
ros2 launch crane_x7_examples banana.launch.py use_sim_time:=true
```

### 3. 実機で実行

```bash
# USBポートの権限設定
sudo chmod 666 /dev/ttyUSB0

# 実機環境でコンテナを起動
docker compose --profile real up --build
```

別ターミナルで以下を実行:

```bash
# コンテナに接続
docker exec -it ros-dev-banana /bin/bash

# ROS 2環境をセットアップ
source /opt/ros/humble/setup.bash
cd /workspace/ros2
source install/setup.bash

# 色分別バナナ仕分けデモを実行する場合
ros2 launch crane_x7_examples color_sorting.launch.py use_sim_time:=false

# または、バナナ仕分けデモを実行する場合
ros2 launch crane_x7_examples banana.launch.py use_sim_time:=false
```

## ドキュメント

| ドキュメント | 説明 |
| --- | --- |
| [セットアップ](docs/setup.md) | 環境構築の詳細手順 |
| [実行方法](docs/usage.md) | シミュレーター/実機での実行方法 |
| [色分別仕分け実装詳細](docs/color-sorting-implementation.md) | 色分別バナナ仕分けデモの実装解説 |
| [実装詳細](docs/implementation.md) | バナナ仕分けデモの実装解説 |
| [開発ガイド](docs/development.md) | 開発環境と新規プログラム追加 |
| [Git運用ルール](docs/git-workflow.md) | コミットメッセージとブランチ運用ルール |
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
