# Git運用ルール

開発をスムーズに進め、コードをきれいに保つために、以下のルールを守りましょう。

## コミットメッセージルール

コミットメッセージは以下のプレフィックスを使用してください：

- **FEAT**: 新機能追加
- **TRIAL**: モデルの調整など、試行錯誤する必要があるコード変更
- **FIX**: バグ修正
- **DOCS**: ドキュメントのみの変更
- **STYLE**: フォーマットの変更、セミコロンの追加などコードの意味に影響しない変更
- **REFACTOR**: バグ修正や機能追加ではないコードの変更
- **PREF**: パフォーマンスを向上させるコード変更
- **TEST**: テストの追加や既存のテストの修正
- **CHORE**: ビルドプロセスや補助ツール、ライブラリの変更

### 例

```bash
git commit -m "FEAT: add imitation learning model"
git commit -m "FIX: resolve robot collision detection issue"
git commit -m "DOCS: update installation guide"
git commit -m "REFACTOR: reorganize MoveIt configuration"
git commit -m "TRIAL: adjust vision detection parameters"
```

## ブランチ運用ルール

### ブランチの役割

リポジトリでは、主に以下のブランチを使います。

#### main (または master)

- **役割**: 本番・リリース用のブランチ
- **ルール**: 常に安定して動作するコードのみを置きます。**絶対に直接コミット（Push）しないこと**

#### develop

- **役割**: 開発の基軸となるブランチ
- **ルール**: 開発中の最新コードがここに集約されます。機能が完成したら、このブランチに合流させます

#### feature/ や fix/ など（作業ブランチ）

- **役割**: 個人の作業用ブランチ
- **ルール**: 新機能の追加やバグ修正など、個々のタスクを行うために使います。必ず `develop` ブランチから作成します
- **例**:
  - `feature/login-function`
  - `feature/color-detection`
  - `fix/header-layout`
  - `fix/gripper-position`

## 基本的な作業フロー

新しい機能を追加する場合、以下のステップで作業を進めます。

### Step 1: 【作業開始】developを最新化し、作業ブランチを作成

```bash
# developブランチに移動
git switch develop

# リモートの最新状態を取得（他の人の作業を取り込む）
git pull origin develop

# 自分の作業ブランチを作成して移動（'task-name'は作業内容がわかる名前に）
git switch -c feature/task-name develop
```

### Step 2: 【作業中】コーディングとコミット

作成した `feature/task-name` ブランチ上で、自由に作業とコミットを繰り返します。

```bash
# (コードを編集...)
git add .
git commit -m "FEAT: add button for XX function"

# (さらに編集...)
git add .
git commit -m "FEAT: implement XX logic"
```

### Step 3: 【作業完了】作業ブランチをリモートにプッシュ

作業が完了したら、リモート（GitHub）に自分の作業ブランチをプッシュします。

```bash
# -u は初回のみ（ローカルとリモートを紐付け）
git push -u origin feature/task-name

# 2回目以降は -u 不要
git push
```

### Step 4: 【レビュー依頼】プルリクエスト (PR) を作成

1. GitHub上で、`feature/task-name` → `develop` へのプルリクエストを作成します
2. タイトルと変更内容を分かりやすく書きます
3. レビュー担当者（チームの誰か）をアサインします

### Step 5: 【レビュー＆マージ】コードレビューとdevelopへの合流

- **（レビュー担当者）** コードを確認し、問題なければ「Approve（承認）」します
- 修正点があれば、GitHub上でコメントします
- **（作業者）** 修正依頼があれば、ローカルの `feature/task-name` ブランチで修正し、再度Push します（Step 2→Step 3を繰り返し）
- **（全員）** 承認されたら、GitHub上でPRをマージします。これで作業内容が `develop` に正式に取り込まれます

### Step 6: 【後片付け】ブランチの削除

マージが完了し、不要になった作業ブランチは削除します。

```bash
# developブランチに戻る
git switch develop

# リモートの最新状態を取得
git pull origin develop

# ローカルの作業ブランチを削除
git branch -d feature/task-name
```

**GitHub上のリモートブランチも、PRマージ時に削除ボタンを押して消します。**

## このルールを守るメリット

- `main` が常に安定し、いつリリースしても安心
- お互いの作業が衝突（コンフリクト）しにくくなる
- コードレビューを必ず挟むことで、コードの品質を保てる
- 変更履歴が整理され、過去の変更を追跡しやすい

## よくある質問

### Q: 複数の機能を同時に開発する場合は？

A: それぞれの機能ごとに別のブランチを作成してください。

```bash
git switch -c feature/function-a develop
# function-a の作業...

git switch develop
git switch -c feature/function-b develop
# function-b の作業...
```

### Q: 急ぎのバグ修正がある場合は？

A: `fix/` プレフィックスを使った作業ブランチを作成し、通常のフローに従います。緊急の場合は、レビュー担当者に優先的に確認してもらうよう依頼してください。

```bash
git switch develop
git pull origin develop
git switch -c fix/urgent-bug develop
# バグ修正...
git commit -m "FIX: resolve critical bug in motion planning"
git push -u origin fix/urgent-bug
# PRを作成してレビュー依頼
```

### Q: コミットメッセージを間違えた場合は？

A: まだプッシュしていなければ、`git commit --amend` で修正できます。

```bash
# 直前のコミットメッセージを修正
git commit --amend -m "FEAT: correct commit message"
```

プッシュ済みの場合は、新しいコミットで対応してください。

## 関連ドキュメント

- [開発ガイド](development.md) - 開発環境と新規プログラム追加
- [トラブルシューティング](troubleshooting.md) - よくある問題と解決方法
