# ライセンスと著作権

## このリポジトリ

- **著作権**: Copyright (c) 2025 ymgchi
- **ライセンス**: MIT License
- **文責**: ymgchi

> このプロジェクトは教育目的で作成されており、MIT License の下で自由に使用・改変・再配布が可能です。

## 外部/サードパーティパッケージ

### Git サブモジュール

- **crane_x7_ros** - RT Corporation: Apache License 2.0
- **crane_x7_description** - RT Corporation: 非商用ライセンス
  - 研究・内部使用のみ許可
  - 商用利用には RT Corporation からの事前許可が必要
- **realsense-ros** - Intel Corporation: Apache License 2.0

> **重要**: RT Corporation のパッケージは、このリポジトリのオリジナルコードとは異なるライセンス条件が適用されます。

## 元リポジトリ

このプロジェクトは [NOPLAB/crane_x7_vla](https://github.com/NOPLAB/crane_x7_vla) をベースにしています。

- **元リポジトリ著作権**: Copyright (c) 2025 nop (NOPLAB)
- **元リポジトリライセンス**: MIT License

## CRANE-X7 関連パッケージ

### crane_x7_ros パッケージ

- **著作権**: Copyright 2022 RT Corporation
- **ライセンス**: Apache License 2.0
- **使用バージョン**: [ymgchi/crane_x7_ros](https://github.com/ymgchi/crane_x7_ros) (humble branch)
- **フォーク元**: [NOPLAB/crane_x7_ros](https://github.com/NOPLAB/crane_x7_ros)
- **大元のリポジトリ**: [rt-net/crane_x7_ros](https://github.com/rt-net/crane_x7_ros)
- **カスタマイズ内容**:
  - banana sorting demo を追加

### crane_x7_description パッケージ

- **著作権**: Copyright 2022 RT Corporation
- **ライセンス**: NON-COMMERCIAL LICENSE
- **使用バージョン**: [ymgchi/crane_x7_description](https://github.com/ymgchi/crane_x7_description) (humble branch)
- **フォーク元**: [rt-net/crane_x7_description](https://github.com/rt-net/crane_x7_description)
- **カスタマイズ内容**:
  - RealSense D435 RGBD カメラセンサーを追加

> **重要**: `crane_x7_description`は**非商用ライセンス**です。商用利用を検討する場合は、RT Corporation に直接お問い合わせください。

### realsense-ros パッケージ

- **著作権**: Copyright 2023 Intel Corporation
- **ライセンス**: Apache License 2.0
- **使用バージョン**: [ymgchi/realsense-ros](https://github.com/ymgchi/realsense-ros) (ros2 branch)
- **フォーク元**: [IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros)
- **用途**: RealSense D435 カメラの ROS 2 ドライバ

### オリジナルデモプログラム

#### banana sorting demo

- **著作権**: Copyright (c) 2025 ymgchi
- **ライセンス**: Apache License 2.0
- **リポジトリ**: [ymgchi/crane_x7_ros](https://github.com/ymgchi/crane_x7_ros) (humble branch)
- **ファイル**:
  - `ros2/src/crane_x7_ros/crane_x7_examples/src/banana.cpp`
  - `ros2/src/crane_x7_ros/crane_x7_examples/launch/banana.launch.py`
  - 関連する CMakeLists.txt、package.xml の変更

#### color sorting demo

- **著作権**: Copyright (c) 2025 ymgchi
- **ライセンス**: Apache License 2.0
- **リポジトリ**: [ymgchi/crane_x7_ros](https://github.com/ymgchi/crane_x7_ros) (humble branch)
- **ファイル**:
  - `ros2/src/crane_x7_ros/crane_x7_examples/src/color_sorting.cpp`
  - `ros2/src/crane_x7_ros/crane_x7_examples/launch/color_sorting.launch.py`
  - 関連する CMakeLists.txt、package.xml の変更
- **特徴**:
  - OpenCV による HSV 色空間での色検出
  - RealSense D435 カメラによる RGB-D センシング
  - TF2 による座標変換
  - 複数物体の一括検出とバッチ処理

#### point cloud sorting demo

- **著作権**: Copyright (c) 2025 ymgchi
- **ライセンス**: Apache License 2.0
- **リポジトリ**: [ymgchi/crane_x7_ros](https://github.com/ymgchi/crane_x7_ros) (humble branch)
- **ファイル**:
  - `ros2/src/crane_x7_ros/crane_x7_examples/scripts/point_cloud_sorting_node.py`
  - `ros2/src/crane_x7_ros/crane_x7_examples/scripts/point_cloud_sorting/` (Python モジュール)
  - `ros2/src/crane_x7_ros/crane_x7_examples/src/motion_service_node.cpp`
  - `ros2/src/crane_x7_ros/crane_x7_examples/launch/point_cloud_sorting.launch.py`
  - 関連する CMakeLists.txt、package.xml の変更
- **特徴**:
  - Python + C++ 分離アーキテクチャ
  - HSV 色検出 + エッジ検出
  - RViz 可視化対応
  - ROS 2 サービスベースのロボット制御

## ライセンス互換性

| コンポーネント                   | ライセンス     | 商用利用 | 改変 | 再配布 |
| -------------------------------- | -------------- | -------- | ---- | ------ |
| このリポジトリ（親）             | MIT            | 可       | 可   | 可     |
| crane_x7_ros (ymgchi フォーク版) | Apache 2.0     | 可       | 可   | 可     |
| banana demo                      | Apache 2.0     | 可       | 可   | 可     |
| color sorting demo               | Apache 2.0     | 可       | 可   | 可     |
| point cloud sorting demo         | Apache 2.0     | 可       | 可   | 可     |
| crane_x7_description (ymgchi フォーク版) | NON-COMMERCIAL | 不可     | 可   | 可     |
| realsense-ros (ymgchi フォーク版) | Apache 2.0     | 可       | 可   | 可     |

詳細は各パッケージの `LICENSE` ファイルを参照してください。
