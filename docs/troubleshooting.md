# トラブルシューティング

## コンテナ関連

### コンテナ起動エラー

**症状**: `docker compose up` でエラーが発生

**対処法**:

```bash
# 既存のコンテナを削除
docker rm -f ros-dev-banana

# イメージを再ビルド
docker compose build --no-cache
```

### X11 表示エラー

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

## USB/実機関連

### USB デバイスが見つからない

**症状**: `/dev/ttyUSB0` が存在しない

**対処法**:

```bash
# デバイスを確認
ls /dev/ttyUSB*
ls /dev/ttyACM*

# 正しいデバイス名が見つかったら .env を更新
# USB_DEVICE=/dev/ttyUSB0
```

### USB 権限エラー

**症状**: "Permission denied" エラー

**対処法**:

```bash
# 一時的な対処
sudo chmod 666 /dev/ttyUSB0

# 恒久的な対処（ユーザーをdialoutグループに追加）
sudo usermod -aG dialout $USER
# ログアウト後に再ログイン
```

## ビルド関連

### colcon build でエラー

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

### パッケージが見つからない

**症状**: `Package 'xxx' not found`

**対処法**:

```bash
# 環境変数を再読み込み
source /workspace/ros2/install/setup.bash

# または、新しいターミナルで再度コンテナに入る
docker exec -it ros-dev-banana /bin/bash
source /workspace/ros2/install/setup.bash
```

## Gazebo 関連

### Gazebo が重い・遅い

**対処法**:

```bash
# GPU加速が有効か確認（コンテナ内で）
nvidia-smi  # GPUが認識されているか確認
```

- グラフィック品質を下げる: Gazebo の設定で影やアンチエイリアシングをオフ

### 物理シミュレーションが不安定

**対処法**:

- タイムステップを小さくする（Gazebo ワールドファイルを編集）
- 物体の質量や摩擦係数を調整
- 速度制限を下げる（コード内の`setMaxVelocityScalingFactor`）

## ROS2 関連

### ノードが起動しない

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

## 関連ドキュメント

- [セットアップ](setup.md) - 初期設定の確認
- [開発ガイド](development.md) - ビルド手順の詳細
