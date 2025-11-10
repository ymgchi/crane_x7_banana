#!/bin/bash

# USB権限チェック
if [ ! -w /dev/ttyUSB0 ]; then
    echo "USB device /dev/ttyUSB0 is not writable."
    echo "Please run: sudo chmod 666 /dev/ttyUSB0"
    exit 1
fi

# ビルドして起動
cd "$(dirname "$0")/.."
docker compose --profile real up --build
