# CRANE-X7 Banana

**未来ロボティクス学科 ロボット設計制作論実習 3**

## 目次

- [概要](#概要)
- [動作環境](#動作環境)
- [セットアップ](#セットアップ)
- [実行方法](#実行方法)
- [バナナ仕分けデモの実装](#バナナ仕分けデモの実装)
- [ライセンス](#ライセンス)

## 概要

CRANE-X7 ロボットアームを使用したピック&プレース動作のデモンストレーションです。
物体を固定位置から 3 箇所（右・中央・左）に順次配置する動作を自動で実行します。

**開発者**: ymgchi Nekomaru TomiKazu-git
**所属**: 未来ロボティクス学科  
**ライセンス**: MIT License

### 使用技術

- ROS 2 Humble
- MoveIt (モーションプランニング)
- Gazebo (物理シミュレーション)
- Docker (環境構築)

### 使用パッケージ

- **crane_x7_ros**: [ymgchi/crane_x7_ros](https://github.com/ymgchi/crane_x7_ros) (Apache License 2.0)
  - NOPLAB 版をフォークし、banana sorting demo を追加
- **crane_x7_description**: [rt-net/crane_x7_description](https://github.com/rt-net/crane_x7_description) (NON-COMMERCIAL LICENSE)

## 動作環境

- Ubuntu 22.04
- Docker
- NVIDIA GPU (推奨)

## セットアップ

### 1. リポジトリのクローン

```bash
git clone --recursive https://github.com/ymgchi/crane_x7_banana.git
cd crane_x7_banana
```

### 2. 環境変数の設定

```bash
cp .env.template .env
```

## 実行方法

### シミュレーターで実行

```bash
xhost +
./scripts/run_sim.sh
```

別のターミナルでバナナデモを実行:

```bash
docker exec -it ros-dev-banana /bin/bash
cd /workspace/ros2
source install/setup.bash
ros2 launch crane_x7_examples banana.launch.py use_sim_time:=true
```

### 実機で実行

```bash
sudo chmod 666 /dev/ttyUSB0
./scripts/run_real.sh
```

別のターミナルでバナナデモを実行:

```bash
docker exec -it ros-dev-banana /bin/bash
cd /workspace/ros2
source install/setup.bash
ros2 launch crane_x7_examples banana.launch.py use_sim_time:=false
```

## バナナ仕分けデモの実装

### プログラム構成

```
crane_x7_examples/
├── src/banana.cpp              # メインプログラム
├── launch/banana.launch.py     # 起動ファイル
├── CMakeLists.txt              # ビルド設定
└── package.xml                 # 依存関係定義
```

### 実装の特徴

1. **MoveIt による軌道計画**

   - `MoveGroupInterface`を使用したアーム制御
   - デカルト空間での直線軌道生成
   - 衝突回避を考慮した経路計画

2. **動作の安全性**

   - 速度・加速度を最大値の 30%に制限
   - 各動作の成功/失敗を確認
   - 段階的な動作（上昇 → 移動 → 下降）

3. **Gazebo 連携**
   - `gazebo_msgs`を使用した物体位置のリセット
   - シミュレーション時間への対応

### 動作フロー

各タスクで以下の動作を実行（計 3 回繰り返し）:

1. グリッパーを開く
2. ピック位置の上方へ移動
3. 下降してピック
4. グリッパーを閉じる
5. 上昇
6. 配置位置の上方へ移動
7. 下降して配置
8. グリッパーを開く
9. 上昇してホーム位置へ

**配置座標:**

- タスク 1: (0.2, 0.15, 0.13) 右側
- タスク 2: (0.2, 0.0, 0.13) 中央
- タスク 3: (0.2, -0.15, 0.13) 左側

### 主要なコード

```cpp
// MoveItインターフェースの初期化
moveit::planning_interface::MoveGroupInterface move_group_arm(node, "arm");
moveit::planning_interface::MoveGroupInterface move_group_gripper(node, "gripper");

// 速度・加速度の制限
move_group_arm.setMaxVelocityScalingFactor(0.3);
move_group_arm.setMaxAccelerationScalingFactor(0.3);

// 目標姿勢の設定
geometry_msgs::msg::Pose target_pose;
target_pose.position.x = 0.2;
target_pose.position.y = 0.0;
target_pose.position.z = 0.13;
// ... 姿勢の設定 ...

// 軌道計画と実行
move_group_arm.setPoseTarget(target_pose);
move_group_arm.move();
```

## ライセンス

### このプロジェクト

- 著作権: Copyright (c) 2025 ymgchi
- ライセンス: MIT License

### 使用パッケージ

#### crane_x7_ros

- 著作権: Copyright 2022 RT Corporation
- ライセンス: Apache License 2.0
- リポジトリ: [ymgchi/crane_x7_ros](https://github.com/ymgchi/crane_x7_ros)

#### crane_x7_description

- 著作権: Copyright 2022 RT Corporation
- ライセンス: NON-COMMERCIAL LICENSE
- リポジトリ: [rt-net/crane_x7_description](https://github.com/rt-net/crane_x7_description)

---

move_group_arm.setMaxAccelerationScalingFactor(0.3); // 加速度スケール（0.1-1.0）

````

#### タスク回数の変更

繰り返し回数を変更する場合:

```cpp
for (int task = 0; task < 3; task++) {  // この数値を変更
````

> **注意**: コードを変更した後は、必ず再ビルドが必要:
>
> ```bash
> cd /workspace/ros2
> colcon build --packages-select crane_x7_examples --symlink-install
> source install/setup.bash
> ```

### トラブルシューティング

#### エラー: `executable 'banana' not found`

**原因**: ビルドが完了していない、または依存関係が不足

**解決方法**:

```bash
cd /workspace/ros2
colcon build --packages-select crane_x7_examples --symlink-install
source install/setup.bash
```

#### エラー: `file 'banana.launch.py' was not found`

**原因**: パッケージがインストールされていない

**解決方法**:

```bash
cd /workspace/ros2
colcon build --symlink-install
source install/setup.bash
```

#### 動作が不安定な場合

以下を確認してください:

- [ ] 速度・加速度のスケーリングファクターが適切か（推奨: 0.2-0.4）
- [ ] 物体の位置や重さが適切か
- [ ] MoveIt の計画失敗メッセージを確認
- [ ] ログで`[ERROR]`や`[WARN]`メッセージを確認

#### Gazebo で物体がリセットされない場合

1. Gazebo サービスの確認:

```bash
ros2 service list | grep set_entity_state
```

2. 物体名の確認（`target_object`と一致しているか）

3. Gazebo が正しく起動しているか確認

#### MoveIt の軌道計画が失敗する

**症状**: "Failed to plan trajectory" というエラー

**対処法**:

- 目標位置がロボットの可動範囲内か確認
- 障害物との干渉がないか確認
- より大きな余裕を持った中間ポイントを追加

## 開発ガイド

### 開発環境のセットアップ

Docker を使用した開発環境の構築手順:

```bash
# 1. X11の権限設定（GUIアプリケーション表示に必要）
xhost +

# 2. Dockerイメージをビルド
./scripts/build.sh

# 3. 開発用コンテナを起動
./scripts/run.sh
```

### コンテナ内での基本操作

```bash
# ワークスペースに移動
cd /workspace/ros2

# パッケージをビルド
colcon build --symlink-install

# 環境変数を読み込む
source install/setup.bash
```

### 利用可能なサンプルプログラム

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

### 実機での実行

実機を使用する場合の標準的な起動コマンド:

```bash
# デモプログラムを起動
ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0

# または、特定のサンプルを実行
ros2 launch crane_x7_examples example.launch.py example:='gripper_control'
```

### 複数ターミナルでの作業

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

### 新しいプログラムの追加

1. **ソースファイルを作成**: `ros2/src/crane_x7_ros/crane_x7_examples/src/your_program.cpp`
2. **CMakeLists.txt に追加**:

```cmake
set(executable_list
  # ... 既存のリスト ...
  your_program
)
```

3. **ビルド**:

```bash
cd /workspace/ros2
colcon build --packages-select crane_x7_examples --symlink-install
source install/setup.bash
```

4. **実行**:

```bash
ros2 run crane_x7_examples your_program
```

### デバッグのヒント

**ログレベルの変更**:

```bash
ros2 launch crane_x7_examples banana.launch.py --log-level debug
```

**特定ノードのログを確認**:

```bash
ros2 run rqt_console rqt_console
```

**トピックの監視**:

```bash
ros2 topic hz /joint_states  # 更新頻度を確認
rqt_graph                    # ノードとトピックの関係を可視化
```

## プロジェクト構造

```
crane_x7_banana/
├── .env.template              # 環境変数のテンプレート
├── .gitignore                 # Git除外設定
├── docker-compose.yml         # Docker Compose設定（simとrealプロファイル）
├── Dockerfile                 # Dockerイメージ定義（ROS 2 Humble）
├── LICENSE                    # MITライセンス
├── README.md                  # このファイル
│
├── scripts/                   # 便利スクリプト集
│   ├── build.sh              # Dockerイメージビルド
│   ├── run.sh                # 開発用コンテナ起動
│   ├── run_real.sh           # 実機用ワンライナー起動
│   ├── run_sim.sh            # シミュレーター用ワンライナー起動
│   └── monitor_gpu.sh        # GPU使用状況モニター
│
└── ros2/                      # ROS 2ワークスペース
    ├── build/                 # ビルド成果物（Git管理外）
    ├── install/               # インストール済みパッケージ（Git管理外）
    ├── log/                   # ビルド・実行ログ（Git管理外）
    │
    └── src/                   # ソースパッケージ
        ├── crane_x7_description/    # ロボットモデル（URDF/Xacro）
        │   ├── config/              # Rviz設定など
        │   ├── launch/              # 起動ファイル
        │   ├── meshes/              # 3Dメッシュファイル
        │   └── urdf/                # ロボット記述ファイル
        │
        └── crane_x7_ros/            # 制御パッケージ群
            ├── crane_x7/            # メタパッケージ
            ├── crane_x7_control/    # ハードウェア制御インターフェース
            │   ├── config/          # コントローラー設定
            │   ├── include/         # ヘッダーファイル
            │   └── src/             # 制御ドライバー
            │
            ├── crane_x7_examples/   # サンプルプログラム
            │   ├── launch/          # 起動ファイル
            │   │   ├── banana.launch.py      # バナナデモ起動
            │   │   ├── demo.launch.py        # 標準デモ起動
            │   │   └── example.launch.py     # サンプル選択起動
            │   ├── src/             # C++ソースコード
            │   │   ├── banana.cpp            # バナナ仕分けプログラム
            │   │   ├── gripper_control.cpp   # グリッパー制御
            │   │   ├── pick_and_place.cpp    # ピック&プレース
            │   │   └── ...                   # その他のサンプル
            │   ├── CMakeLists.txt   # ビルド設定
            │   └── package.xml      # パッケージ依存関係
            │
            ├── crane_x7_gazebo/     # Gazeboシミュレーター設定
            │   ├── launch/          # シミュレーター起動ファイル
            │   ├── models/          # Gazeboモデル（テーブルなど）
            │   └── worlds/          # Gazeboワールドファイル
            │
            └── crane_x7_moveit_config/  # MoveIt設定
                ├── config/              # 運動学、軌道計画設定
                ├── launch/              # MoveIt起動ファイル
                └── srdf/                # セマンティックロボット記述
```

**重要なディレクトリ:**

- `ros2/src/crane_x7_ros/crane_x7_examples/`: カスタムプログラムの追加先
- `docker-compose.yml`: 実機/シミュレーターの切り替え設定
- `scripts/`: よく使う操作を簡単に実行

## トラブルシューティング

### 一般的な問題

#### コンテナ起動エラー

**症状**: `docker compose up` でエラーが発生

colcon build --symlink-install
**対処法**:

1. 既存のコンテナを削除:

```bash
docker rm -f ros-dev-banana
```

2. イメージを再ビルド:

```bash
docker compose build --no-cache
```

#### X11 表示エラー

**症状**: Gazebo や Rviz が表示されない、"cannot open display" エラー

**対処法**:

```bash
# ホスト側で実行
xhost +

# DISPLAYの確認
echo $DISPLAY  # :0 や :1 が表示されるはず

# .envファイルを確認
cat .env  # DISPLAY=:0 が設定されているか
```

### USB/実機関連

#### USB デバイスが見つからない

**症状**: `/dev/ttyUSB0` が存在しない

**対処法**:

```bash
# デバイスを確認
ls /dev/ttyUSB*
ls /dev/ttyACM*

# 正しいデバイス名が見つかったら .env を更新
# USB_DEVICE=/dev/ttyUSB0
```

#### USB 権限エラー

**症状**: "Permission denied" エラー

**対処法**:

```bash
# 一時的な対処
sudo chmod 666 /dev/ttyUSB0

# 恒久的な対処（ユーザーをdialoutグループに追加）
sudo usermod -aG dialout $USER
# ログアウト後に再ログイン
```

### ビルド関連

#### colcon build でエラー

**症状**: パッケージのビルドに失敗

**対処法**:

```bash
# ビルドキャッシュをクリア
cd /workspace/ros2
rm -rf build/ install/ log/

# 依存関係を再インストール
rosdep install -r -y -i --from-paths src

# 再ビルド
colcon build --symlink-install
```

#### パッケージが見つからない

**症状**: `Package 'xxx' not found`

**対処法**:

```bash
# 環境変数を再読み込み
source /workspace/ros2/install/setup.bash

# または、新しいターミナルで再度コンテナに入る
docker exec -it ros-dev-banana /bin/bash
source /workspace/ros2/install/setup.bash
```

### Gazebo 関連

#### Gazebo が重い・遅い

**対処法**:

- GPU 加速が有効か確認:

```bash
# コンテナ内で
nvidia-smi  # GPUが認識されているか確認
```

- グラフィック品質を下げる: Gazebo の設定で影やアンチエイリアシングをオフ

#### 物理シミュレーションが不安定

**対処法**:

- タイムステップを小さくする（Gazebo ワールドファイルを編集）
- 物体の質量や摩擦係数を調整
- 速度制限を下げる（コード内の`setMaxVelocityScalingFactor`）

### ROS2 関連

#### ノードが起動しない

**症状**: `ros2 launch` でノードが起動しない

**対処法**:

```bash
# ノードの状態を確認
ros2 node list

# トピックの流れを確認
ros2 topic list
ros2 topic hz /joint_states

# ログを確認
ros2 run rqt_console rqt_console
```

## 環境変数設定

`.env` ファイルで以下の変数を設定できます:

```bash
# 実機用: USB デバイスパス（デフォルト）
USB_DEVICE=/dev/ttyUSB0

# シミュレーター用: ディスプレイ設定
DISPLAY=:0
```

**カスタマイズ例:**

```bash
# 異なるUSBポートを使用する場合
USB_DEVICE=/dev/ttyACM0

# マルチディスプレイ環境の場合
DISPLAY=:1
```

## ライセンスと著作権

### このリポジトリ

- **著作権**: Copyright (c) 2025 ymgchi
- **ライセンス**: MIT License
- **文責**: ymgchi

> このプロジェクトは教育目的で作成されており、MIT License の下で自由に使用・改変・再配布が可能です。

### 元リポジトリ

このプロジェクトは [NOPLAB/crane_x7_vla](https://github.com/NOPLAB/crane_x7_vla) をベースにしています。

- **元リポジトリ著作権**: Copyright (c) 2025 nop (NOPLAB)
- **元リポジトリライセンス**: MIT License

### CRANE-X7 ハードウェア・ソフトウェア

CRANE-X7 ロボットアームおよび関連する ROS 2 パッケージは株式会社アールティが開発・提供しています。

#### crane_x7_ros パッケージ

- **著作権**: Copyright 2022 RT Corporation
- **ライセンス**: Apache License 2.0
- **使用バージョン**: [ymgchi/crane_x7_ros](https://github.com/ymgchi/crane_x7_ros) (humble branch)
- **フォーク元**: [NOPLAB/crane_x7_ros](https://github.com/NOPLAB/crane_x7_ros)
- **大元のリポジトリ**: [rt-net/crane_x7_ros](https://github.com/rt-net/crane_x7_ros)
- **カスタマイズ内容**:
  - banana sorting demo を追加
  - ROS 2 Humble 対応（NOPLAB フォーク版ベース）

#### crane_x7_description パッケージ

- **著作権**: Copyright 2022 RT Corporation
- **ライセンス**: NON-COMMERCIAL LICENSE
- **リポジトリ**: [rt-net/crane_x7_description](https://github.com/rt-net/crane_x7_description)
- **使用方法**: Git サブモジュールとして管理

> **重要**: `crane_x7_description`は**非商用ライセンス**です。商用利用を検討する場合は、RT Corporation に直接お問い合わせください。

#### banana sorting demo（このプロジェクトのオリジナル）

- **著作権**: Copyright (c) 2025 ymgchi
- **ライセンス**: Apache License 2.0
- **リポジトリ**: [ymgchi/crane_x7_ros](https://github.com/ymgchi/crane_x7_ros) (humble branch)
- **ファイル**:
  - `ros2/src/crane_x7_ros/crane_x7_examples/src/banana.cpp`
  - `ros2/src/crane_x7_ros/crane_x7_examples/launch/banana.launch.py`
  - 関連する CMakeLists.txt、package.xml の変更

### ライセンス互換性

| コンポーネント                   | ライセンス     | 商用利用 | 改変 | 再配布 | 備考                  |
| -------------------------------- | -------------- | -------- | ---- | ------ | --------------------- |
| このリポジトリ（親）             | MIT            | 可       | 可   | 可     | -                     |
| crane_x7_ros (ymgchi フォーク版) | Apache 2.0     | 可       | 可   | 可     | NOPLAB フォークベース |
| banana demo                      | Apache 2.0     | 可       | 可   | 可     | オリジナル            |
| crane_x7_description             | NON-COMMERCIAL | 不可     | 可   | 可     | 非商用のみ            |

詳細は各パッケージの `LICENSE` ファイルを参照してください。

## 参考情報

### 公式ドキュメント

- **CRANE-X7 製品ページ**: https://rt-net.jp/products/crane-x7/
- **RT-net GitHub**: https://github.com/rt-net

### ROS 2 & MoveIt

- **ROS 2 Humble Documentation**: https://docs.ros.org/en/humble/
- **MoveIt 2 Documentation**: https://moveit.ros.org/
- **Gazebo Documentation**: https://gazebosim.org/

### CRANE-X7 関連リポジトリ

| リポジトリ                      | 説明                                   | URL                                            |
| ------------------------------- | -------------------------------------- | ---------------------------------------------- |
| crane_x7                        | メインリポジトリ                       | https://github.com/rt-net/crane_x7             |
| crane_x7_ros                    | ROS 2 パッケージ（公式）               | https://github.com/rt-net/crane_x7_ros         |
| crane_x7_ros（NOPLAB フォーク） | ROS 2 パッケージ（NOPLAB 版）          | https://github.com/NOPLAB/crane_x7_ros         |
| crane_x7_ros（ymgchi フォーク） | ROS 2 パッケージ（banana demo 追加版） | https://github.com/ymgchi/crane_x7_ros         |
| crane_x7_description            | ロボットモデル                         | https://github.com/rt-net/crane_x7_description |
| crane_x7_hardware               | ハードウェア情報                       | https://github.com/rt-net/crane_x7_Hardware    |
| crane_x7_samples                | Python サンプル集                      | https://github.com/rt-net/crane_x7_samples     |

---

**Last Updated**: 2025 年 11 月 17 日  
**Version**: 1.0  
**Author**: ymgchi
