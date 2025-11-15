#!/bin/bash
# GPU使用状況を監視するスクリプト

echo "GPU使用状況を監視します（Ctrl+Cで終了）"
echo "RVizでロボットを動かしてみてください"
echo ""
echo "時刻,温度(℃),GPU利用率(%),メモリ(MiB),電力(W)"
echo "================================================"

while true; do
    nvidia-smi --query-gpu=timestamp,temperature.gpu,utilization.gpu,memory.used,power.draw --format=csv,noheader,nounits
    sleep 1
done
