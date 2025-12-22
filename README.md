# CRANE-X7 Banana

**未来ロボティクス学科 ロボット設計制作論実習3 - チームBanana**

CRANE-X7 ロボットアームを使用したビジョンベースのピック＆プレースデモンストレーション

## デモ

https://github.com/user-attachments/assets/6d47b2b3-5b5c-4eec-bcbe-505cfa42c7e4

## 概要

RealSenseカメラのRGB-D画像を使用して、オレンジ・黄・緑の3色のキューブを検出し、色ごとに異なる場所へ自動仕分けします。

- **オレンジ色キューブ**: 右奥に配置
- **黄色キューブ**: 左奥に配置
- **緑色キューブ**: 前方に配置

### 主な機能

- RGB画像からHSV色空間による色検出
- 深度画像を用いた3次元位置推定
- TF2による座標変換（カメラ座標系→ロボット座標系）
- MoveItによるモーションプランニング
- 5個のキューブをランダムな色で自動スポーン（シミュレーション）

## クイックスタート

```bash
# リポジトリをクローン
git clone --recursive https://github.com/ymgchi/crane_x7_banana.git
cd crane_x7_banana
cp .env.template .env

# シミュレーター起動
xhost +
docker compose --profile sim up

# 別ターミナルでデモ実行
docker exec -it ros-dev-banana /bin/bash
source /opt/ros/humble/setup.bash && source /workspace/ros2/install/setup.bash
ros2 launch crane_x7_examples color_sorting.launch.py use_sim_time:=true
```

詳細は [セットアップガイド](docs/setup.md) を参照してください。

## ドキュメント

| ドキュメント | 説明 |
| --- | --- |
| [セットアップ](docs/setup.md) | 環境構築の詳細手順 |
| [実行方法](docs/usage.md) | シミュレーター/実機での実行方法 |
| [色分別デモ実装詳細](docs/color-sorting-implementation.md) | 色分別キューブ仕分けデモの実装解説 |
| [開発ガイド](docs/development.md) | 開発環境と新規プログラム追加 |
| [Git運用ルール](docs/git-workflow.md) | コミットメッセージとブランチ運用ルール |
| [トラブルシューティング](docs/troubleshooting.md) | よくある問題と解決方法 |
| [ライセンス](docs/license.md) | ライセンスと著作権情報 |
| [参考情報](docs/references.md) | 関連リンク集 |

## 使用技術

- ROS 2 Humble
- MoveIt (モーションプランニング)
- Gazebo (物理シミュレーション)
- OpenCV (色検出・画像処理)
- RealSense D435 カメラ (RGB-D センシング)
- Docker (環境構築)

## 今後の課題

- 点群データを用いたエッジ検出による把持点推定にも取り組みましたが、カメラを真上から見下ろす角度ではエッジが検出されにくい課題があり、改善を検討中です

## 謝辞

このプロジェクトは [NOPLAB/crane_x7_vla](https://github.com/NOPLAB/crane_x7_vla) をベースにしています。
- 元リポジトリ著作権: Copyright (c) 2025 nop (NOPLAB)
- 元リポジトリライセンス: MIT License

## ライセンス

- **このリポジトリ**: Apache License 2.0 (Copyright (c) 2025 ymgchi)
- **crane_x7_ros**: Apache License 2.0
- **crane_x7_description**: NON-COMMERCIAL LICENSE

詳細は [docs/license.md](docs/license.md) を参照してください。
