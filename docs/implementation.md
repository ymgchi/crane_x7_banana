# バナナ仕分けデモの実装

## プログラム構成

```
crane_x7_examples/
├── src/banana.cpp              # メインプログラム
├── launch/banana.launch.py     # 起動ファイル
├── CMakeLists.txt              # ビルド設定
└── package.xml                 # 依存関係定義
```

## 実装の特徴

### 1. MoveIt による軌道計画

- `MoveGroupInterface`を使用したアーム制御
- デカルト空間での直線軌道生成
- 衝突回避を考慮した経路計画

### 2. 動作の安全性

- 速度・加速度を最大値の 70%に制限
- 各動作の成功/失敗を確認
- 段階的な動作（上昇 → 移動 → 下降）

### 3. Gazebo 連携

- `gazebo_msgs`を使用した物体位置のリセット
- シミュレーション時間への対応

## 動作フロー

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

- タスク 1: (0.35, 0.20, 0.12) 右側
- タスク 2: (0.35, 0.0, 0.12) 中央
- タスク 3: (0.35, -0.20, 0.12) 左側

## 主要なコード

```cpp
// MoveItインターフェースの初期化
moveit::planning_interface::MoveGroupInterface move_group_arm(node, "arm");
moveit::planning_interface::MoveGroupInterface move_group_gripper(node, "gripper");

// 速度・加速度の制限
move_group_arm.setMaxVelocityScalingFactor(0.7);
move_group_arm.setMaxAccelerationScalingFactor(0.7);

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

## 関連ドキュメント

- [開発ガイド](development.md) - 新しいプログラムの追加方法
- [実行方法](usage.md) - デモの実行手順
