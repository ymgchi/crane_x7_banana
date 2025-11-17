# CRANE-X7 Banana 🍌

**未来ロボティクス学科 ロボット設計制作論実習3**

## 📋 目次

- [概要](#概要)
- [主な機能](#主な機能)
- [必要環境](#必要環境)
- [クイックスタート](#クイックスタート)
- [バナナ仕分けデモ](#バナナ仕分けデモ)
- [開発ガイド](#開発ガイド)
- [プロジェクト構造](#プロジェクト構造)
- [サブモジュールの管理](#サブモジュールの管理)
- [トラブルシューティング](#トラブルシューティング)
- [ライセンスと著作権](#ライセンスと著作権)
- [参考情報](#参考情報)

## 概要

このリポジトリは [NOPLAB/crane_x7_vla](https://github.com/NOPLAB/crane_x7_vla) をベースに、VLA/機械学習関連の機能を削除し、CRANE-X7 ロボットアームの基本制御に特化したものです。

**開発者**: ymgchi  
**所属**: 未来ロボティクス学科  
**用途**: ロボット設計制作論実習3  
**ライセンス**: MIT License

### 使用しているサブモジュール

このリポジトリは以下の外部パッケージをGitサブモジュールとして使用しています:

- **crane_x7_ros**: [NOPLAB/crane_x7_ros](https://github.com/NOPLAB/crane_x7_ros) (Apache License 2.0)

- **crane_x7_description**: [rt-net/crane_x7_description](https://github.com/rt-net/crane_x7_description) (NON-COMMERCIAL LICENSE)

## 主な機能

- **Gazeboシミュレーター対応**: 実機なしでも動作確認可能
- **MoveIt統合**: 高度な軌道計画とモーションプランニング
- **バナナ仕分けデモ**: 物体のピック＆プレース動作のサンプル実装
- **Docker完全対応**: 環境構築が簡単、依存関係の問題なし
- **GPU加速サポート**: NVIDIAグラフィックスによる高速レンダリング

## 必要環境

| 項目 | 要件 | 備考 |
|------|------|------|
| **OS** | Ubuntu 22.04 (Native Linux) | WSL2での動作は未検証 |
| **Docker** | Docker Engine 20.10+ | Docker Compose V2対応 |
| **GPU** | NVIDIA GPU (推奨) | Gazeboの3D描画に使用 |
| **ハードウェア** | CRANE-X7 ロボットアーム | 実機使用時のみ必要 |
| **USBポート** | /dev/ttyUSB0 | 実機接続時に使用 |

## クイックスタート

### 1. リポジトリのクローン

```bash
git clone --recursive https://github.com/ymgchi/crane_x7_banana.git
cd crane_x7_banana
```

> **注意**: `--recursive` オプションを忘れずに！サブモジュールが正しくクローンされます。

### 2. 環境変数ファイルの作成

```bash
# .env.template をコピーして .env を作成
cp .env.template .env
```

`.env` ファイルの内容:
```bash
# 実機用: USB デバイスパス
USB_DEVICE=/dev/ttyUSB0

# シミュレーター用: ディスプレイ設定
DISPLAY=:0
```

通常はこのままで問題ありません。

### 3. 実行方法の選択

####  シミュレーター（Gazebo）で実行【推奨・初心者向け】

実機なしでも動作確認できます。

```bash
# X11の権限設定
xhost +

# ビルド＆起動
./scripts/run_sim.sh
```

Gazeboウィンドウが開き、ロボットアームとテーブルが表示されます。

####  実機で実行

**前提条件**: CRANE-X7をUSBで接続し、十分な作業スペースを確保してください。

```bash
# USB デバイスの権限設定
sudo chmod 666 /dev/ttyUSB0

# ビルド＆起動
./scripts/run_real.sh
```

**安全上の注意**: 
- ロボットが動作する範囲に障害物がないことを確認
- 緊急停止できるよう準備
- 初回は低速で動作を確認

## バナナ仕分けデモ

### 概要

バナナ仕分けデモは、CRANE-X7ロボットアームを使用して物体を固定位置から3箇所に自動仕分けするプログラムです。MoveItを使用した軌道計画とグリッパー制御を組み合わせることで、ピック＆プレース動作を実現しています。

**特徴:**
-  固定位置の物体を3箇所（右・中央・左）に順次配置
-  デカルト空間での直線軌道生成（Cartesian Path Planning）
-  安全性を考慮した速度・加速度制限（30%）
-  Gazeboシミュレーターでの物体位置自動リセット機能

### ファイル構成

```
ros2/src/crane_x7_ros/crane_x7_examples/
├── src/
│   └── banana.cpp              # メインプログラム（C++実装）
├── launch/
│   └── banana.launch.py        # 起動ファイル
├── CMakeLists.txt              # ビルド設定
└── package.xml                 # パッケージ依存関係定義
```

**主要ファイルの役割:**
- **banana.cpp**: MoveItの`MoveGroupInterface`を使用してアームとグリッパーを制御
- **banana.launch.py**: ロボットの記述情報とMoveIt設定を読み込み、`use_sim_time`パラメータで実機/シミュレーション切り替え

### 実行方法

####  方法1: シミュレーター（Gazebo）で実行【推奨】

**ステップ1: Gazeboシミュレーターを起動**

```bash
# ターミナル1
./scripts/run_sim.sh
```

Gazeboが起動し、ロボットアームとテーブルが表示されます。

**ステップ2: バナナ仕分けデモを実行**

新しいターミナルを開いて以下を実行:

```bash
# ターミナル2
docker exec -it ros-dev-banana /bin/bash
cd /workspace/ros2
source install/setup.bash
ros2 launch crane_x7_examples banana.launch.py use_sim_time:=true
```

> **ヒント**: 初回実行時や`banana.cpp`を変更した場合は、ビルドが必要です:
> ```bash
> cd /workspace/ros2
> colcon build --packages-select crane_x7_examples --symlink-install
> source install/setup.bash
> ```

####  方法2: 実機で実行

**注意**: 周囲の安全を確認し、物体の位置を適切に配置してください。

```bash
# USB デバイスの権限設定
sudo chmod 666 /dev/ttyUSB0

# 実機を接続してコンテナを起動
./scripts/run_real.sh
```

新しいターミナルから:

```bash
docker exec -it ros-dev-banana /bin/bash
cd /workspace/ros2
source install/setup.bash
ros2 launch crane_x7_examples banana.launch.py use_sim_time:=false
```

####  方法3: Docker Composeで直接起動

シミュレーター:
```bash
xhost +
docker compose --profile sim up --build
```

実機:
```bash
sudo chmod 666 /dev/ttyUSB0
docker compose --profile real up --build
```

### 動作フロー

各タスク（合計3回）で以下のステップを順次実行:

| ステップ | 動作内容 | 詳細 |
|---------|---------|------|
| 1 | グリッパーを開く | 物体を掴む準備 (60度) |
| 2 | ピック上方位置へ移動 | 物体の25cm上に移動 |
| 3 | 下降 | 直線軌道で物体の位置まで下降 (13cm) |
| 4 | グリッパーを閉じる | 物体を掴む (20度) |
| 5 | 上昇 | 物体を持ち上げる (25cm) |
| 6 | 配置上方位置へ移動 | 目標位置の上に移動 |
| 7 | 下降 | 直線軌道で配置位置まで下降 |
| 8 | グリッパーを開く | 物体を離す |
| 9 | 上昇 | 配置位置から離れる |
| 10 | ホーム位置へ戻る | 初期姿勢に戻る |
| 11 | 物体リセット | 次のタスクのため元の位置に戻す (Gazeboのみ) |

**配置位置:**
- タスク1: 右側 (Y = +0.15m)
- タスク2: 中央 (Y = 0.0m)
- タスク3: 左側 (Y = -0.15m)

### カスタマイズ

#### 位置の変更

`banana.cpp` の座標を編集することで、ピック＆プレース位置を調整できます:

```cpp
// 座標の定義（X, Y, Z, Roll, Pitch, Yaw）
auto pick_pose = createPose(0.2, 0.0, 0.13, -180, 0, -90);           // ピック位置
auto place_pose_1 = createPose(0.2, 0.15, 0.13, -180, 0, -90);      // 配置位置1（右）
auto place_pose_2 = createPose(0.2, 0.0, 0.13, -180, 0, -90);       // 配置位置2（中央）
auto place_pose_3 = createPose(0.2, -0.15, 0.13, -180, 0, -90);     // 配置位置3（左）
```

#### 速度・加速度の調整

動作速度を変更する場合:

```cpp
move_group_arm.setMaxVelocityScalingFactor(0.3);      // 速度スケール（0.1-1.0）
move_group_arm.setMaxAccelerationScalingFactor(0.3);  // 加速度スケール（0.1-1.0）
```

#### タスク回数の変更

繰り返し回数を変更する場合:

```cpp
for (int task = 0; task < 3; task++) {  // この数値を変更
```

> **注意**: コードを変更した後は、必ず再ビルドが必要:
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
- [ ] MoveItの計画失敗メッセージを確認
- [ ] ログで`[ERROR]`や`[WARN]`メッセージを確認

#### Gazeboで物体がリセットされない場合

1. Gazeboサービスの確認:
```bash
ros2 service list | grep set_entity_state
```

2. 物体名の確認（`target_object`と一致しているか）

3. Gazeboが正しく起動しているか確認

#### MoveItの軌道計画が失敗する

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

| サンプル名 | 説明 | 実行コマンド |
|-----------|------|-------------|
| `gripper_control` | グリッパーの開閉制御 | `ros2 launch crane_x7_examples example.launch.py example:='gripper_control'` |
| `pose_groupstate` | 事前定義されたポーズへの移動 | `ros2 launch crane_x7_examples example.launch.py example:='pose_groupstate'` |
| `joint_values` | 関節角度を指定した制御 | `ros2 launch crane_x7_examples example.launch.py example:='joint_values'` |
| `pick_and_place` | ピック＆プレース動作 | `ros2 launch crane_x7_examples example.launch.py example:='pick_and_place'` |
| `cartesian_path` | デカルト空間での軌道追従 | `ros2 launch crane_x7_examples example.launch.py example:='cartesian_path'` |

**Gazeboで実行する場合**: `use_sim_time:='true'` を追加
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

## サブモジュールの管理

このプロジェクトはGitサブモジュールを使用しています。サブモジュール内のファイル（`banana.cpp`など）を変更した場合の管理方法を説明します。

### 🔍 現在の状況

`crane_x7_ros`と`crane_x7_description`はサブモジュールとして外部リポジトリを参照しています：

```bash
# サブモジュールの確認
cat .gitmodules
```

### 選択肢1：サブモジュールの変更を無視【推奨・簡単】

サブモジュール内の変更（`banana.cpp`など）をGit管理から除外する方法：

```bash
# サブモジュールの変更を無視する設定
git config submodule.ros2/src/crane_x7_ros.ignore dirty

# 確認（"modified: ros2/src/crane_x7_ros" が表示されなくなる）
git status
```

**メリット**:
- ✅ 設定が簡単（1コマンドで完了）
- ✅ `git status`が常にクリーン
- ✅ サブモジュールの更新が楽

**デメリット**:
- ❌ `banana.cpp`などの変更がGitHubにバックアップされない
- ❌ 他の環境で同期できない

**この方法が向いている人**: 
- 個人開発で、ローカルだけで動けばOK
- サブモジュールの元リポジトリを変更したくない

### 選択肢2：サブモジュールをフォークして管理【完全管理】

自分のGitHubアカウントにサブモジュールをフォークして、完全に管理する方法：

#### ステップ1: GitHubでフォークを作成

1. https://github.com/NOPLAB/crane_x7_ros にアクセス
2. 右上の「Fork」ボタンをクリック
3. 自分のアカウントにフォークを作成

#### ステップ2: サブモジュールを自分のフォークに変更

```bash
cd ~/dev/my-crane-x7-project

# サブモジュール内に移動
cd ros2/src/crane_x7_ros

# 現在のリモートを確認
git remote -v

# 自分のフォークをリモートとして追加
git remote add myfork https://github.com/ymgchi/crane_x7_ros.git

# 新しいブランチを作成
git checkout -b my-banana-changes

# 変更をコミット
git add .
git commit -m "Add banana.cpp and custom modifications"

# 自分のフォークにプッシュ
git push myfork my-banana-changes

# 親リポジトリに戻る
cd ~/dev/my-crane-x7-project
```

#### ステップ3: .gitmodulesを更新

```bash
# .gitmodulesを編集（テキストエディタで開く）
nano .gitmodules
```

以下のように変更:
```ini
[submodule "ros2/src/crane_x7_ros"]
	path = ros2/src/crane_x7_ros
	url = https://github.com/ymgchi/crane_x7_ros.git  # 自分のフォークに変更
	branch = my-banana-changes  # 自分のブランチに変更
```

#### ステップ4: サブモジュールの参照を更新

```bash
# サブモジュールの設定を同期
git submodule sync

# サブモジュールを更新
git submodule update --remote --merge

# 変更をコミット
git add .gitmodules ros2/src/crane_x7_ros
git commit -m "Update submodule to use my fork"
git push
```

**メリット**:
- ✅ すべての変更がGitHubにバックアップされる
- ✅ 他の環境でも同じ状態を再現可能
- ✅ 変更履歴が管理される

**デメリット**:
- ❌ 設定が少し複雑
- ❌ 元のNOPLABリポジトリの更新を取り込むのに手間がかかる

**この方法が向いている人**:
- チームで開発している
- 複数のマシンで同じコードを使いたい
- 変更履歴をしっかり管理したい

### 選択肢3：サブモジュールを通常のディレクトリに変換【中間案】

サブモジュールを解除して、通常のファイルとして管理する方法：

```bash
cd ~/dev/my-crane-x7-project

# サブモジュールの登録を解除
git submodule deinit -f ros2/src/crane_x7_ros
git rm -f ros2/src/crane_x7_ros

# .git/modulesから削除
rm -rf .git/modules/ros2/src/crane_x7_ros

# 通常のディレクトリとしてコピー（バックアップから復元）
# または、元のリポジトリを通常のcloneとして追加
cd ros2/src
git clone https://github.com/NOPLAB/crane_x7_ros.git
rm -rf crane_x7_ros/.git  # Gitディレクトリを削除

# 親リポジトリに追加
cd ~/dev/my-crane-x7-project
git add ros2/src/crane_x7_ros
git commit -m "Convert crane_x7_ros from submodule to regular directory"
git push
```

**メリット**:
- ✅ すべてのファイルが1つのリポジトリで管理される
- ✅ 設定が簡単、サブモジュールの複雑さがない

**デメリット**:
- ❌ 元のリポジトリの更新を取り込むのが困難
- ❌ リポジトリサイズが大きくなる
- ⚠️ ライセンス上の注意が必要（派生物として明記）

### 🎯 推奨：どれを選ぶべきか

| 状況 | 推奨方法 | 理由 |
|------|---------|------|
| 個人学習・実習用 | **選択肢1（無視）** | 最もシンプル、十分に機能する |
| 複数環境で使用 | **選択肢2（フォーク）** | 変更が同期される |
| チーム開発 | **選択肢2（フォーク）** | 共同作業が可能 |
| 元リポジトリと無関係に開発 | **選択肢3（通常化）** | 独立して管理 |

**あなたの場合（実習用）**: **選択肢1**が最適です！

```bash
# これだけ実行すればOK！
git config submodule.ros2/src/crane_x7_ros.ignore dirty
git status  # クリーンになっているはず
```

### 📝 補足：banana.cppはサブモジュール内でも動作します

**Q: banana.cppはサブモジュール内の位置でないと動かない？**

**A: はい、その場所でないと動きません。** 理由：

1. **ビルドシステムの参照**: `CMakeLists.txt`が`ros2/src/crane_x7_ros/crane_x7_examples/src/`を参照
2. **パッケージ構造**: ROS 2のパッケージ構造に従っている
3. **依存関係**: 同じパッケージ内の他のファイルに依存

**でも大丈夫！** サブモジュール内のファイルを変更しても問題なく動作します。Git管理の問題だけです。

### 🔧 実際の作業フロー（選択肢1を選んだ場合）

```bash
# 1. サブモジュールの変更を無視する設定（1回だけ）
git config submodule.ros2/src/crane_x7_ros.ignore dirty

# 2. 通常通り開発
# banana.cppを編集して保存

# 3. 親リポジトリの変更だけコミット
git add README.md .env scripts/
git commit -m "Update documentation"
git push

# 4. サブモジュールの警告は無視される
git status  # クリーン！
```

これで安心して開発できます！🎉

## トラブルシューティング

### 一般的な問題

#### コンテナ起動エラー

**症状**: `docker compose up` でエラーが発生

**対処法**:
1. 既存のコンテナを削除:
```bash
docker rm -f ros-dev-banana
```

2. イメージを再ビルド:
```bash
docker compose build --no-cache
```

3. ディスク容量を確認:
```bash
df -h
docker system df
```

#### X11表示エラー

**症状**: GazeboやRvizが表示されない、"cannot open display" エラー

**対処法**:
```bash
# ホスト側で実行
xhost +

# DISPLAYの確認
echo $DISPLAY  # :0 や :1 が表示されるはず

# .envファイルを確認
cat .env  # DISPLAY=:0 が設定されているか
```

**WSL2の場合**:
```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

### USB/実機関連

#### USBデバイスが見つからない

**症状**: `/dev/ttyUSB0` が存在しない

**対処法**:
```bash
# デバイスを確認
ls /dev/ttyUSB*
ls /dev/ttyACM*

# 正しいデバイス名が見つかったら .env を更新
# USB_DEVICE=/dev/ttyUSB0
```

#### USB権限エラー

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

### Gazebo関連

#### Gazeboが重い・遅い

**対処法**:
- GPU加速が有効か確認:
```bash
# コンテナ内で
nvidia-smi  # GPUが認識されているか確認
```

- グラフィック品質を下げる: Gazeboの設定で影やアンチエイリアシングをオフ

#### 物理シミュレーションが不安定

**対処法**:
- タイムステップを小さくする（Gazeboワールドファイルを編集）
- 物体の質量や摩擦係数を調整
- 速度制限を下げる（コード内の`setMaxVelocityScalingFactor`）

### ROS2関連

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

#### MoveItの軌道計画が失敗する

**症状**: "Failed to compute motion plan" エラー

**対処法**:
- 目標位置が可動範囲内か確認
- 衝突がないか確認（Rvizで可視化）
- planning timeを増やす:
```cpp
move_group.setPlanningTime(10.0);  // 秒単位
```
- 異なるプランナーを試す（設定ファイルで変更）

### デバッグテクニック

#### 詳細ログの有効化

```bash
# より詳細なログを出力
ros2 launch crane_x7_examples banana.launch.py --log-level debug

# 特定のノードのみログ出力
ros2 run crane_x7_examples banana --ros-args --log-level debug
```

#### ROS2ツールの活用

```bash
# ノードの情報を表示
ros2 node info /banana_sorting_node

# トピックの型を確認
ros2 topic info /joint_states

# サービスの呼び出し
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{}"

# パラメータの確認
ros2 param list
ros2 param get /move_group robot_description
```

### よくある質問（FAQ）

**Q: Dockerなしで実行できますか？**  
A: はい。ただし、ROS 2 Humble、MoveIt、Gazebo等の依存関係を手動でインストールする必要があります。

**Q: Windows/macOSで動きますか？**  
A: Dockerは動作しますが、実機接続とGUI表示に追加設定が必要です。WSL2推奨。

**Q: 他のロボットアームでも使えますか？**  
A: URDFとMoveIt設定を変更すれば可能ですが、CRANE-X7専用に最適化されています。

**Q: 商用利用は可能ですか？**  
A: MITライセンスなので可能ですが、crane_x7_descriptionは非商用ライセンスです。詳細は各ライセンスを確認してください。

## 環境変数設定

`.env` ファイルで以下の変数を設定できます:

```bash
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

> このプロジェクトは教育目的で作成されており、MIT Licenseの下で自由に使用・改変・再配布が可能です。

### 元リポジトリ

このプロジェクトは [NOPLAB/crane_x7_vla](https://github.com/NOPLAB/crane_x7_vla) をベースにしています。

- **元リポジトリ著作権**: Copyright (c) 2025 nop (NOPLAB)
- **元リポジトリライセンス**: MIT License

### CRANE-X7 ハードウェア・ソフトウェア

CRANE-X7 ロボットアームおよび関連するROS 2パッケージは株式会社アールティが開発・提供しています。

#### crane_x7_ros パッケージ

- **著作権**: Copyright 2022 RT Corporation
- **ライセンス**: Apache License 2.0
- **リポジトリ**: [rt-net/crane_x7_ros](https://github.com/rt-net/crane_x7_ros) （このプロジェクトでは[NOPLABフォーク版](https://github.com/NOPLAB/crane_x7_ros)を使用）

#### crane_x7_description パッケージ

- **著作権**: Copyright 2022 RT Corporation
- **ライセンス**: NON-COMMERCIAL LICENSE
- **リポジトリ**: [rt-net/crane_x7_description](https://github.com/rt-net/crane_x7_description)

> **⚠️ 重要**: `crane_x7_description`は**非商用ライセンス**です。商用利用を検討する場合は、RT Corporationに直接お問い合わせください。

### ライセンス互換性

| コンポーネント | ライセンス | 商用利用 | 改変 | 再配布 |
|---------------|-----------|----------|------|--------|
| このリポジトリ | MIT | ✅ | ✅ | ✅ |
| crane_x7_ros | Apache 2.0 | ✅ | ✅ | ✅ |
| crane_x7_description | NON-COMMERCIAL | ❌ | ✅ | ✅ (非商用) |

詳細は各パッケージの `LICENSE` ファイルを参照してください。

## 参考情報

### 公式ドキュメント

- **CRANE-X7 製品ページ**: https://rt-net.jp/products/crane-x7/
- **RT-net GitHub**: https://github.com/rt-net

### ROS 2 & MoveIt

- **ROS 2 Humble Documentation**: https://docs.ros.org/en/humble/
- **MoveIt 2 Documentation**: https://moveit.ros.org/
- **Gazebo Documentation**: https://gazebosim.org/

### CRANE-X7関連リポジトリ

| リポジトリ | 説明 | URL |
|-----------|------|-----|
| crane_x7 | メインリポジトリ | https://github.com/rt-net/crane_x7 |
| crane_x7_ros | ROS 2パッケージ（公式） | https://github.com/rt-net/crane_x7_ros |
| crane_x7_ros（フォーク） | ROS 2パッケージ（NOPLAB版） | https://github.com/NOPLAB/crane_x7_ros |
| crane_x7_description | ロボットモデル | https://github.com/rt-net/crane_x7_description |
| crane_x7_hardware | ハードウェア情報 | https://github.com/rt-net/crane_x7_Hardware |
| crane_x7_samples | Pythonサンプル集 | https://github.com/rt-net/crane_x7_samples |

### コミュニティとサポート

- **RT-net フォーラム**: 製品サポートページ参照
- **ROS Answers**: https://answers.ros.org/
- **Stack Overflow**: タグ `ros2`, `moveit`, `gazebo`

### 追加リソース

- **YouTube**: RT-netの公式チャンネルでCRANE-X7のデモ動画を視聴可能
- **技術ブログ**: RT-netの技術ブログでロボット開発のヒントを入手

---

##  学習リソース

### 初心者向け

1. **ROS 2 入門**: [ROS 2チュートリアル（日本語）](https://docs.ros.org/en/humble/Tutorials.html)
2. **MoveIt 入門**: [MoveIt 2 Tutorials](https://moveit.picknik.ai/humble/index.html)
3. **Docker 入門**: [Docker公式ドキュメント](https://docs.docker.com/)

### 中級者向け

1. **軌道計画**: MoveItの高度な機能（衝突回避、把持計画など）
2. **センサー統合**: RealSenseカメラとの統合
3. **カスタム制御**: 独自の制御アルゴリズムの実装

---

**Last Updated**: 2025年11月15日  
**Version**: 1.0  
**Author**: ymgchi
