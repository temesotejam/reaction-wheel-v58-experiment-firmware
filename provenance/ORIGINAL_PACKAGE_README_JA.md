# V58 最新実装・動作解説・研究の現在地

作成日: 2026-09-19（日本時間）。対象revision: `paired_probe_coast_v58_50ms_20260913`。

**最初に `START_HERE.html` をブラウザで開いてください。インターネット接続なしで解説を読めます。** ZIPを展開してから開くと、ソース・資料へのリンクも利用できます。

## このパッケージの内容

| 場所 | 内容 |
| --- | --- |
| [動作解説](docs/OPERATION_GUIDE_JA.html) | センサ取得、姿勢推定、動的β、歩行制御の計算、V58測定の状態遷移、電流積分、停止、記録、操作 |
| [経緯と現在地](docs/CURRENT_STATUS_AND_HISTORY_JA.html) | 動的βの検証から歩行成立、Q到達精度、車輪状態依存、V58設計まで。実測済み・未検証・次の作業を区別 |
| `firmware/` | V58のPlatformIOプロジェクト。全製品ソース31件、設定、既存説明、V58試験・解析とその依存スクリプト |
| `binary/firmware.bin` | 2026-09-13の検証済みアプリケーションイメージ。ビルド記録とSHA-256を照合して同梱 |
| `evidence/` | 元のビルド・書込み状態・ソフト試験の記録、今回の再試験、ソース参照位置 |
| `references/` | 2026-09-15の研究経緯PDF（8頁）と実測時系列PDF（10頁） |
| `PACKAGE_MANIFEST.json` | 全同梱ファイルのサイズ・SHA-256と入力元の対応 |
| `VERIFY_PACKAGE.py` | 展開後に全ファイルのハッシュを照合するPythonスクリプト |

解説はMarkdown原稿も `docs/` に同梱しています。HTMLとMarkdownは同じ内容です。

## 何が最新で、実機はどこまで進んだか

V58は最新の**実装済み測定ファームウェア**です。PROBE（±300 mA）とCOAST（Current Mode 0 mA）を比較します。姿勢推定と旧歩行制御のソースも含みますが、V58のWeb操作では歩行開始APIが無効です。ZIPをそのまま書き込んで±8度歩行を開始する構成ではありません。

V58のビルド・ソフト試験は成功していますが、保存された書込み記録は**USB再接続待ち・未書込み・実機未測定**です。最後に書込みと実測を確認した版はV57bです。これは作業履歴に基づく状態で、今回実機を接続して読み出した結果ではありません。

今回、ファームウェアの動作変更・マイコン書込み・測定開始は行っていません。元の47ファイルを元ビルドmanifestおよび作業プロジェクトとバイト照合し、解説を追加しています。付属の再試験は模擬I2Cと合成データによるソフト試験です。

## ソース・資料の原本

- 作業プロジェクト: `Projects/2026_09_13_atoms3cam_v58_paired_probe_coast_logger/`
- 実装正本: `study/06_code/v58_paired_probe_coast_20260913/`
- 元のビルド・試験: `study/07_results/v58_implementation_flash_20260913/`
- 今回の解説・梱包コード: `study/06_code/v58_explained_package_20260919/`
- 今回の検証記録: `study/07_results/v58_explained_package_20260919/`

ワークスペースは `C:/Users/arika/Desktop/workspace_research`。ZIP中のリンクは展開先で読める相対リンクを使います。元manifestに記載されたパスは当時の原本位置です。

## 動作・検証の入口

```powershell
# ZIPを展開したトップフォルダで、全同梱ファイルを照合
python -B VERIFY_PACKAGE.py

# ソースをビルド。PlatformIOと依存ライブラリが必要
cd firmware
pio run -e atoms3cam

# 模擬試験。Python、NumPy、Matplotlib、g++が必要
python -B tools/test_v58.py

# 実機で取得した新しいV58ログを解析
python -B tools/analyze_v58.py run1.rwlog run2.rwlog --out analysis/my_v58_runs
```

`firmware/docs/INDEX_V58.html` は実機Web画面の写しです。解説の入口は `START_HERE.html` です。写しをPCから直接開いても実機の測定操作はできません。

`binary/firmware.bin` はアプリ単体であり、統合フラッシュイメージではありません。通常は `firmware/` からPlatformIOでビルド・書込みます。手動書込み用のアドレス推測を避けるため、ブートローダ・パーティションの単独書込み手順は本パッケージに含めません。

## 同梱しなかったもの・保管

`.git/`、`.pio/`全体、ツールチェーン、第三者ライブラリのキャッシュ、旧版の未使用試験群、実測RWLOG・動画・大容量生データ、合成データの全出力は同梱していません。PlatformIO依存バージョンは `firmware/platformio.ini`、元ビルドの情報は `evidence/v58_original/` にあります。初回ビルドには依存取得が必要になることがあります。

このZIPは受渡し用です。受領・別途保管を確認した後、または後継パッケージへ置き換えた後に `_exchange/` のコピーを削除できます。ソース・解説・検証の正本は保持します。今回は既存の他の受渡し物は削除していません。
