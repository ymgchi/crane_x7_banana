#!/bin/bash

# X11権限設定
xhost + 2>/dev/null

# 既存のコンテナを削除（存在する場合）
echo "既存のコンテナをクリーンアップ中..."
docker rm -f ros-dev-banana 2>/dev/null && echo "コンテナを削除しました" || echo "削除するコンテナがありません"

# ビルドして起動
cd "$(dirname "$0")/.."
docker compose --profile sim up --build
