# V58 Paired Probe / Coast Control

V57bの固定パルス応答から自然減速を差し引くための測定プログラムです。[受領仕様](docs/V58_USER_SPEC.md)に基づく派生版です。revision: `paired_probe_coast_v58_50ms_20260913`。

## 測定

Wi-Fi `AtomS3CAM_Q1_SHADOW`（パスワード `12345678`）、ブラウザ `http://192.168.4.1/`。画面タイトルは **V58 Paired Probe / Coast** です。

- Small: 5速度領域 × 2方向 × PROBE/COAST、各1有効、合計20。最大40試行。
- Full: 同20条件で各5有効、合計100。最大200試行、時間予算10分。
- 各速度・方向でPROBE/COASTを隣接して予定し、条件ブロックごとに先行役割を交互にします。有効数を満たした条件は再試行を省略するため、未達時には片側だけの追加測定もあります。
- 各イベントは独立したbaseline、準備回転、Current Mode 0 mAへの移行、30 ms連続の残留電流判定、fresh current/speed/Vbus確認を経ます。速度モードと電流モードは順番に切り替えます。
- PROBE: 指定方向300 mA、公称60 ms。COAST: Current Mode 0 mAを維持。どちらもCURRENTレジスタ書込みの完了をイベント開始にします。COASTの書込み値は0で、非ゼロ電流を指令しません。これは通電したドライバの電流モード0での対照であり、電源を切った自然回転とは同一とは限りません。
- baseline未成立、移行未達、速度領域未達、速度50 ms観測の遅れは、その試行の無効として記録します。電流/速度通信失敗、電流stale、電圧・速度上限、出力/モード不整合、停止期限異常は停止します。
- 機体を固定し、電源と接続を確認してから測定してください。測定中は電流記録優先でWeb応答が待つため、緊急停止はAtom本体ボタンまたはモータ電源遮断を使用してください。条件不足でもRWLOGを保存してください。

## 時刻とログ

ファイル名 `paired_probe_v58_run_*.rwlog`、binary format 48、metadata `rwlog_paired_probe_v58`、`paired_probe_v58`/`q_observer_v58`。最大200試行、74列の配列形式で列名は`trial_columns`にあります。

| 項目 | 意味 |
| --- | --- |
| `is_coast`, `probe_direction` | COAST識別と共通の符号d（COASTでは仮想方向） |
| `pair_id` | 予定上の同速度・方向ペア。初期状態の一致を保証しない |
| `event_start_us` | PROBE/COAST共通の開始時刻 |
| `nonzero_write_*` | PROBEのみ実時刻。COASTでは0 |
| `coast_event_start_us` | COASTのみ実時刻。PROBEでは0 |
| `speed_trace` | 開始前と開始後10/20/30/40/50 msの速度。各点は実読取り完了時刻とrpm |
| `speed_50ms_time_us` | 50 msを過ぎて最初の予定読取りの完了時刻。有効範囲は開始から50～52 ms |
| `speed_observation_dt_us` | 開始前速度読取りから50 ms側速度読取りまでの実時間差 |
| `aligned_speed_delta_50_rpm` | d ×（50 ms側速度 − 開始前速度） |
| `valid_event` | 実機上のbaseline/transfer/鮮度/速度観測/停止/電流タイミング合格 |

時刻はmicrosの32 bit循環値、I2C読取り完了時刻です。センサ内部のサンプリング時刻ではありません。速度は厳密な0 msと50 msでの同時観測とは呼ばず、実観測時刻と差を保存します。主Q50は実電流の**開始0～50 ms**積分で、隣接した有効点の補間だけを使います。

互換的な内部クラス/ファイル名にV57が残ります。`before_probe`と`age_at_nonzero_end`を含む旧列名は両役割のイベント前・開始時鮮度を表す別名です。`actual_probe_width_us`は両役割の観測開始から終了0書込み完了まで。旧`aligned_speed_delta_rpm`と`q_probe_60ms_on_device_mA_s`は60 ms終了付近の補助値で、主解析では使いません。`observation_count`はCOASTも数えます。

fresh current→Q更新→速度読取りの順を守り、状態レジスタ群の途中にも期限を処理します。電流gap上限3333 usは維持します。一般時系列は10 Hz、電流はイベント前・中・後の全fresh sample、速度は専用traceを使います。最大試行記録はPSRAMへ置き、確保できない場合は測定開始を拒否します。

## 解析

```powershell
python -B tools/analyze_v58.py run1.rwlog run2.rwlog --out analysis/my_runs
```

Python、NumPy、Matplotlibが必要です。解析品質はbaseline/transfer、実電流の端点・欠測・gap、開始時刻、役割、50 ms速度traceと実時間差を再確認します。合格数と同条件のマッチ成立数を分けて出力します。

同Run・同方向・同実速度領域内で一対一マッチします。初期差の上限は速度20 rpm、aligned current 0.1 mA、Vbus 0.05 V、速度観測dt 500 us。許容辺を各上限で規格化した距離の小さい順に採用し、一度使った対照は再利用しません。これは貪欲法であり最大全体マッチ数を保証する最適割当ではありません。全試行に最近傍の差と候補数、未成立理由を残し、条件を自動緩和しません。

paired値は `dΔω_probe − dΔω_coast`、Gはその値/Q50です。Gは純粋なトルク定数ではありません。一次結果は実測速度差を保持し、時間差は500 us以内に制限します。補助の50 ms正規化値は実速度変化率×0.05秒という近似です。

もう一つの方法は、Run・方向別のCOAST実速度変化率を `[1, speed, clip(speed/75,-1,1)]` で回帰し、PROBEの初期速度へ評価した変化率×PROBE実観測dtを差し引きます。最低8COAST・3速度領域・フルランク、速度支持範囲内、近傍current/Vbus支持がある場合だけ算出します。回帰形と時間スケーリングは解析上の仮定です。実測支持範囲外の外挿はしません。

`V58_TRIALS.csv`、`V58_MATCHED_PAIRS.csv`、`V58_COAST_REGRESSION.csv`、`V58_METHOD_COMPARISON.csv`、`V58_COVERAGE.csv`、`V58_PAIR_COVERAGE.csv`、`V58_SPEED_TRACES.csv`、`V58_SUMMARY.json`と6図を出力します。6図は速度対PROBE差分、COAST差分、補正後差分、Q50、G、および負・ゼロ・正速度での対応ペア時系列です。代表ペアがなければ「No matched example」と明示します。

2 Run以上で方向別の再現性、マッチ不足、残留電流と電圧の交絡、pairedと回帰法の差を確認し、仕様のCase A/B/Cを判断します。測定数不足だけで解析は停止しません。モデル採用や制御変更は自動で行いません。

## 検証・ビルド

```powershell
python -B tools/test_v58.py
C:/Users/arika/.platformio/penv/Scripts/platformio.exe run
C:/Users/arika/.platformio/penv/Scripts/platformio.exe run -t upload --upload-port COM7
```

native試験にはg++が必要です。製品C++の完走/未達/異常/停止境界/速度遅延と、最大metadata、合成RWLOGの解析解・対応・回帰・欠測/CRC等を試験します。合成試験は実機の成立を証明しません。コピー元の旧テストはV58の試験入口ではありません。

2026-09-13時点でビルドと試験は成功。ユーザーが別作業後にUSB再接続を知らせる予定のため、書込みと通常起動、実機測定は未実施です。旧V57bプロジェクトと実機ファームウェアはこの時点では維持されています。
