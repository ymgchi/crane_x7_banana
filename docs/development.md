# 開発ガイド

## 開発環境のセットアップ

Docker を使用した開発環境の構築手順:

```bash
# 1. X11の権限設定（GUIアプリケーション表示に必要）
xhost +

# 2. Dockerイメージをビルド
./scripts/build.sh

# 3. 開発用コンテナを起動
./scripts/run.sh
```

## コンテナ内での基本操作

```bash
# ワークスペースに移動
cd /workspace/ros2

# パッケージをビルド
colcon build --symlink-install

# 環境変数を読み込む
source install/setup.bash
```

## 複数ターミナルでの作業

別ターミナルから実行中のコンテナに接続:

```bash
# コンテナに接続
docker exec -it ros-dev-banana /bin/bash

# 環境変数を読み込む
source /workspace/ros2/install/setup.bash

# ROS2コマンドを実行
ros2 topic list                    # トピック一覧を表示
ros2 node list                     # ノード一覧を表示
ros2 topic echo /joint_states      # トピックの内容を表示
```

## 新しいプログラムの追加

### 1. ソースファイルを作成

`ros2/src/crane_x7_ros/crane_x7_examples/src/your_program.cpp`

### 2. CMakeLists.txt に追加

```cmake
set(executable_list
  # ... 既存のリスト ...
  your_program
)
```

### 3. ビルド

```bash
cd /workspace/ros2
colcon build --packages-select crane_x7_examples --symlink-install
source install/setup.bash
```

### 4. 実行

```bash
ros2 run crane_x7_examples your_program
```

## デバッグのヒント

### ログレベルの変更

```bash
ros2 launch crane_x7_examples banana.launch.py --log-level debug
```

### 特定ノードのログを確認

```bash
ros2 run rqt_console rqt_console
```

### トピックの監視

```bash
ros2 topic hz /joint_states  # 更新頻度を確認
rqt_graph                    # ノードとトピックの関係を可視化
```

## プロジェクト構造

```
crane_x7_banana/
├── .env.template              # 環境変数のテンプレート
├── docker-compose.yml         # Docker Compose設定
├── Dockerfile                 # Dockerイメージ定義
├── scripts/                   # 便利スクリプト集
│   ├── build.sh              # Dockerイメージビルド
│   ├── run.sh                # 開発用コンテナ起動
│   ├── run_real.sh           # 実機用起動
│   └── run_sim.sh            # シミュレーター用起動
│
└── ros2/                      # ROS 2ワークスペース
    └── src/
        ├── crane_x7_description/    # ロボットモデル（URDF/Xacro）
        └── crane_x7_ros/            # 制御パッケージ群
            ├── crane_x7_control/    # ハードウェア制御
            ├── crane_x7_examples/   # サンプルプログラム
            ├── crane_x7_gazebo/     # Gazebo設定
            └── crane_x7_moveit_config/  # MoveIt設定
```

**重要なディレクトリ:**

- `ros2/src/crane_x7_ros/crane_x7_examples/`: カスタムプログラムの追加先
- `docker-compose.yml`: 実機/シミュレーターの切り替え設定
- `scripts/`: よく使う操作を簡単に実行

## 関連ドキュメント

- [実装詳細](implementation.md) - バナナデモの実装解説
- [Git運用ルール](git-workflow.md) - コミットメッセージとブランチ運用ルール
- [トラブルシューティング](troubleshooting.md) - 問題解決
