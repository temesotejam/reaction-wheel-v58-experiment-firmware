# Reaction Wheel V58 Experiment Firmware

リアクションホイール機体の入力応答を、**300 mA PROBE** と **0 mA COAST** の対照測定で切り分けるための V58 実験ファームウェアです。

- Revision: `paired_probe_coast_v58_50ms_20260913`
- Target: M5Stack AtomS3CAM / ESP32-S3 (`m5stack-atoms3`)
- Framework: Arduino + PlatformIO
- V58 status: **実装・ソフト検証済み / 実機書込み・実測は未実施**
- 最後に記録上実機へ書き込まれた版: `fixed_probe_v57b_baseline_mad_q50_20260912`

> このリポジトリは「V58で歩行制御が実機成功済み」であることを示すものではありません。V58は、歩行制御へ戻す前にモータ入力と自然な車輪速度変化を分離するための測定版です。

## 目的

V57bでは、50 ms窓の電流積分量 `Q50` は高い再現性を示した一方、同じ300 mA入力でも初期Wheel速度によって観測速度変化が大きく変わりました。ただし観測値にはモータ作用だけでなく自然減速も含まれます。

V58では、近い初期状態で次の2条件を比較します。

- **PROBE**: ±300 mA、公称60 ms
- **COAST**: Current Mode 0 mAのまま同じ時間軸で観測

主解析は開始後 `0–50 ms` に固定し、PROBEとCOASTの速度変化差からモータ寄与候補を評価します。

## 最初に読むもの

1. [`docs/OPERATION_GUIDE_JA.md`](docs/OPERATION_GUIDE_JA.md) — センサ、姿勢推定、V58状態遷移、Q積分、停止、ログ、操作
2. [`docs/CURRENT_STATUS_AND_HISTORY_JA.md`](docs/CURRENT_STATUS_AND_HISTORY_JA.md) — 研究経緯、実測済み事項、未検証事項、次の作業
3. [`firmware/README.md`](firmware/README.md) — V58測定仕様と解析方法
4. [`firmware/docs/V58_USER_SPEC.md`](firmware/docs/V58_USER_SPEC.md) — V58受領仕様
5. [`ARTIFACTS.md`](ARTIFACTS.md) — 元受渡しパッケージに含まれていた生成物・参照PDFのhash

## リポジトリ構成

```text
.
├─ firmware/          PlatformIOプロジェクトと解析・試験ツール
│  ├─ src/            ファームウェア本体
│  ├─ tools/          RWLOG変換、V57/V57b/V58解析、模擬試験
│  └─ docs/           V58仕様と保存版Web資料
├─ docs/              動作解説・研究経緯・構成図
├─ evidence/          元ビルド、試験、書込み状態、再梱包試験の証跡
├─ binary/README.md   元BINのhashと扱い
├─ references/README.md  元参照PDFのhashと扱い
├─ provenance/        元受渡しパッケージのmanifest/検証スクリプト
└─ ARTIFACTS.md
```

## GitHub版で生成物を分離した理由

元の受渡しZIPは、ソースだけでなく `firmware.bin` と参照PDFまで含む「引継ぎパッケージ」でした。一方、このGitHubリポジトリでは**再現可能なソースと検証証跡を正本**とし、次の3ファイルはGit履歴へ重複格納していません。

- `binary/firmware.bin`
- `references/2026-09-15_dynamic_beta_to_walking_report_ja.pdf`
- `references/2026-09-15_current_status_timeseries_ja.pdf`

サイズとSHA-256は [`ARTIFACTS.md`](ARTIFACTS.md) に固定しています。元パッケージのmanifestも [`provenance/ORIGINAL_PACKAGE_MANIFEST.json`](provenance/ORIGINAL_PACKAGE_MANIFEST.json) に保存してあります。

## ソフト試験

Python、NumPy、Matplotlib、g++が必要です。

```bash
cd firmware
python -B tools/test_v58.py
python -B tools/test_v58_offline.py
```

2026-09-19の再確認では、製品C++の状態遷移試験と合成RWLOGによるオフライン解析試験はいずれもPASSしています。

## ビルド

PlatformIOが必要です。

```bash
cd firmware
pio run -e atoms3cam
```

主要依存は `firmware/platformio.ini` に固定されています。

```text
platform = espressif32@6.7.0
M5Unified = 0.2.18
Adafruit AHRS = 2.4.0
```

GitHub Actionsでも同じ環境を使ってソフト試験とPlatformIOビルドを実行します。

## V58測定の概要

Smallは20有効イベント、Fullは100有効イベントを目標とします。5速度領域 × 2方向 × PROBE/COASTを測定し、各試行で独立baseline、速度準備、Current Mode 0 mAへの移行、fresh current/speed/Vbus確認を行います。

速度は開始前と開始後10/20/30/40/50 msに記録します。主Q値はPROBEの方向付きraw currentを開始0–50 msで台形積分した `Q50` です。PROBE/COASTのペアは予定 `pair_id` だけでは決めず、同Run・同方向・同実速度領域内で初期速度、開始前電流、Vbus、速度観測dtが近いものを一対一で対応させます。

## 現在の重要な制約

- V58の実機書込み・実測成功はまだ記録されていません。
- COASTはドライバを無通電にした自由回転ではなく、**Current Mode 0 mA** の対照です。
- V58の固定パルス状態は歩行側の動的βパルス状態へ接続されていないため、通常のV58測定中は採用姿勢フィルタがβ=0.025を維持します。
- V58解析結果から入力モデル採用や歩行制御変更を自動では行いません。
- 実機では、Web停止だけに依存せずAtom本体ボタンまたはモータ電源遮断を緊急停止手段として確保してください。

## 次の実機作業

1. 対象個体と接続ポートを確認してV58を書き込む。
2. revision、Wi-Fi、通常起動、書込みhashを確認する。
3. 固定した機体でV58 Fullを2 Run以上取得する。
4. CRC、電流gap、停止期限、50 ms速度点、条件別有効数、実状態マッチ数を監査する。
5. paired法とCOAST回帰法で自然変化差引き後の再現性を比較する。
6. 支持される入力モデルが得られた場合のみ、別の歩行用派生版へ反映する。

## 由来と証跡

- [`evidence/CURRENT_STATE.json`](evidence/CURRENT_STATE.json)
- [`evidence/SOURCE_MAP.json`](evidence/SOURCE_MAP.json)
- [`evidence/v58_original/`](evidence/v58_original/)
- [`evidence/repack/`](evidence/repack/)
- [`provenance/ORIGINAL_PACKAGE_MANIFEST.json`](provenance/ORIGINAL_PACKAGE_MANIFEST.json)

研究全体のシミュレーションやモデル検証とは役割を分け、このリポジトリはV58実験ファームウェアとその再現・引継ぎ資料に限定します。
