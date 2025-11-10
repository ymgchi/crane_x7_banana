#!/bin/bash

# X11権限設定
xhost + 2>/dev/null

# ビルドして起動
cd "$(dirname "$0")/.."
docker compose --profile sim up --build
