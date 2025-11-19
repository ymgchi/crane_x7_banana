# セットアップ

## 動作環境

- Ubuntu 22.04
- Docker
- NVIDIA GPU (推奨)

## 1. リポジトリのクローン

```bash
git clone --recursive https://github.com/ymgchi/crane_x7_banana.git
cd crane_x7_banana
```

## 2. 環境変数の設定

```bash
cp .env.template .env
```

### 環境変数の詳細

`.env` ファイルで以下の変数を設定できます:

```bash
# 実機用: USB デバイスパス（デフォルト）
USB_DEVICE=/dev/ttyUSB0

# シミュレーター用: ディスプレイ設定
DISPLAY=:0
```

**カスタマイズ例:**

```bash
# 異なるUSBポートを使用する場合
USB_DEVICE=/dev/ttyACM0

# マルチディスプレイ環境の場合
DISPLAY=:1
```

## 3. Docker イメージのビルド

```bash
./scripts/build.sh
```

## 次のステップ

- [実行方法](usage.md) - シミュレーターまたは実機でデモを実行
- [開発ガイド](development.md) - 開発環境の詳細設定
