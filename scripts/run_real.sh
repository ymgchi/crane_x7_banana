#!/bin/bash

# USB権限チェック
if [ ! -w /dev/ttyUSB0 ]; then
    echo "USB device /dev/ttyUSB0 is not writable."
    echo "Please run: sudo chmod 666 /dev/ttyUSB0"
    exit 1
fi

# 既存のコンテナを削除（存在する場合）
echo "既存のコンテナをクリーンアップ中..."
docker rm -f ros-dev-banana 2>/dev/null && echo "コンテナを削除しました" || echo "削除するコンテナがありません"

# ビルドして起動
cd "$(dirname "$0")/.."
docker compose --profile real up --build
