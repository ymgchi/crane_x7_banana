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
source /opt/ros/humble/setup.bash
cd /workspace/ros2
source install/setup.bash
ros2 launch crane_x7_examples color_sorting.launch.py use_sim_time:=true
```

デモが開始すると:
1. 5個のキューブがランダムな色・位置で自動スポーン
2. カメラで全キューブをスキャン
3. 各キューブをビジュアルサーボイングで高精度に位置合わせ
4. 色ごとに指定の場所へ配置

## 実機で実行

### 1. USB権限を設定して起動

```bash
sudo chmod 666 /dev/ttyUSB0
docker compose --profile real up
```

### 2. 色分別キューブ仕分けデモを実行

別のターミナルで:

```bash
docker exec -it ros-dev-banana /bin/bash
source /opt/ros/humble/setup.bash
cd /workspace/ros2
source install/setup.bash
ros2 launch crane_x7_examples color_sorting.launch.py use_sim_time:=false
```

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
