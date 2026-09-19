# RW Walker — V58 Paired Probe / Coast Control

## 目的

V57bでは、

* fixed probe測定基盤：成立
* \(Q_{\rm probe,50}\)：非常に高い再現性
* Wheel初期速度による観測上の \(\Delta\omega\) 差：明確
* Wheel速度と残留currentの完全な分離：未達

となった。

特に、

$$
Q_{\rm probe,50}\approx4.10\ {\rm mA\cdot s}
$$

はほぼ一定なのに、同じ300 mA probeに対するWheel速度変化は初期速度によって大きく異なった。

しかしWheelはpulseを与えなくても自然減速する。

したがって現在の

$$
\Delta\omega_{\rm observed}
$$

には、

$$
\text{モータによる速度変化}
+
\text{自然減速}
$$

が混在している。

V58では、

**同じ初期Wheel状態で「300 mA probe」と「0 mA coast」を比較し、自然減速を差し引いたモータ作用だけを求める。**

---

# 1. 実験条件を2種類にする

同じ初期状態について、以下の2条件を測定する。

## PROBE

```text
300 mA
nominal 60 ms
```

V57bと同じ。

## COAST

```text
0 mAのまま
```

で、probeと同じ観測時間だけWheelを自由回転させる。

COASTは「何もしない」のではなく、

**PROBEと同じタイミングで開始イベントを発生させ、同じ時刻で速度を観測する対照実験**

とする。

---

# 2. 主解析時間を50 msに固定する

V57bと同じ理由で、60 ms終了点を主解析には使わない。

主解析区間：

$$
\boxed{0\rightarrow50{\rm ms}}
$$

とする。

PROBEでは、

$$
Q_{\rm probe,50}
=
\int_0^{50{\rm ms}}i_{\rm dir}(t)\,dt
$$

を使用する。

COASTでも全く同じ50 msについてWheel速度変化を求める。

これにより60 ms pulse終了処理の時間ばらつきを主解析から除外する。

---

# 3. イベント開始時刻

## PROBE

$$
t_0
=
\text{nonzero CURRENT write end}
$$

## COAST

0 mAを維持したまま、

```text
coast_event_start_us
```

を明示的に生成する。

COASTでもPROBEと同じ、

* fresh speed
* fresh current
* bus voltage

取得後に \(t_0\) を確定する。

---

# 4. 初期状態

各イベント直前に必ず、

```text
rw_speed_before
current_before
bus_voltage
```

を取得する。

probe方向を \(d=\pm1\) として、

$$
\omega_{\rm aligned,0}
=
d\,\omega_{\rm RW,0}
$$

を使用する。

COASTにも仮想的に同じ \(d\) を割り当てる。

つまりCOASTでも、

$$
d\Delta\omega_{\rm coast}
$$

を計算可能にする。

---

# 5. 速度領域

最低限、

$$
d\omega_0
\approx
-300,\ -150,\ 0,\ +150,\ +300\ {\rm rpm}
$$

程度を確保する。

完全一致させる必要はなく、解析には実測値を使用する。

---

# 6. PROBEとCOASTをペアにする

同じ速度領域について、

```text
PROBE
COAST
```

の両方を測る。

可能なら、

```text
PROBE → COAST
COAST → PROBE
```

の順序を交互にする。

常にPROBEを先、COASTを後にはしない。

温度・電圧・時間経過との交絡を防ぐ。

---

# 7. baseline / transfer処理

V57bで成立したbaseline・transfer処理を基本的に維持する。

ただし前trialの影響が残った状態で次イベントへ進まない。

各イベント前に、

```text
baseline_valid
transfer_ready
fresh_speed_valid
fresh_current_valid
```

を要求する。

---

# 8. 残留currentは引き続き保存する

probe直前の、

$$
i_{\rm before,aligned}
=
d\,i_{\rm before}
$$

を保存する。

V58では「相関を完全に0にする」ことだけを目的にはしない。

代わりにPROBEとCOASTを、

* aligned speed
* residual current
* bus voltage

が近い組み合わせで比較できるようにする。

---

# 9. Wheel speedを50 ms時点で取得する

各イベントについて最低限、

```text
speed_before_event
speed_at_50ms
```

を取得する。

実際のtimestampも保存する。

理想的に50.000 msへ完全一致しない場合は、

$$
\Delta t_\omega
=
t_{\omega,50}-t_{\omega,0}
$$

も保存する。

必要に応じて、

$$
\frac{\Delta\omega}{\Delta t}
$$

でも比較できるようにする。

---

# 10. PROBE中のcurrent timingを壊さない

50 ms speed readを行う場合も、

```text
fresh current
→ 必要なQ更新
→ wheel speed read
```

の順を基本とする。

正式監査：

```text
current read failure = 0
active current gap > 3.333 ms = 0
```

を維持する。

---

# 11. 観測速度変化

PROBE：

$$
\Delta\omega_{\rm probe}
=
\omega_{\rm probe,50}
-
\omega_{\rm probe,0}
$$

方向正規化：

$$
\Delta\omega_{\rm probe,dir}
=
d\Delta\omega_{\rm probe}
$$

COAST：

$$
\Delta\omega_{\rm coast}
=
\omega_{\rm coast,50}
-
\omega_{\rm coast,0}
$$

$$
\Delta\omega_{\rm coast,dir}
=
d\Delta\omega_{\rm coast}
$$

とする。

---

# 12. V58の最重要量

モータ入力による正味のWheel速度変化を、

$$
\boxed{
\Delta\omega_{\rm motor}
=
\Delta\omega_{\rm probe,dir}
-
\Delta\omega_{\rm coast,dir}
}
$$

とする。

これにより、

**同じ初期速度で自然に起こる速度変化を差し引いた、300 mA probeによる追加効果**

を求める。

---

# 13. 完全に同じrpmのペアが無い場合

offlineでmatched comparisonを行う。

各PROBEに対して、

```text
aligned speed
current_before_aligned
bus voltage
```

が最も近いCOASTを対応付ける。

最低限、対応差を保存する。

```text
delta_initial_speed
delta_initial_current
delta_bus_voltage
```

あまりに離れた組み合わせはmatched pairとして使用しない。

---

# 14. ペア解析と回帰解析を両方行う

## Paired analysis

$$
\Delta\omega_{\rm motor}
$$

を直接求める。

## Regression analysis

COASTについて、

$$
\Delta\omega_{\rm coast,dir}
=
f(d\omega_0)
$$

を求める。

そのcoast curveからPROBE初期速度における自然減速を推定し、

$$
\Delta\omega_{\rm motor}
=
\Delta\omega_{\rm probe,dir}
-
\widehat{\Delta\omega}_{\rm coast,dir}
$$

も計算する。

両方法が同じ傾向になるか確認する。

---

# 15. Qとの対応

PROBEについて、

$$
Q_{50}
$$

と、

$$
\Delta\omega_{\rm motor}
$$

を比較する。

特に、

$$
\boxed{
G_{\omega Q}
=
\frac{\Delta\omega_{\rm motor}}{Q_{50}}
}
$$

を診断量として求める。

これは、

> current readback上で同じQを投入したとき、Wheelへどれだけ正味の速度変化が生じたか

を見る量である。

まだ純粋なトルク定数とは呼ばない。

---

# 16. 最重要グラフ

## Graph 1

$$
d\omega_0
$$

vs

$$
\Delta\omega_{\rm probe,dir}
$$

---

## Graph 2

$$
d\omega_0
$$

vs

$$
\Delta\omega_{\rm coast,dir}
$$

---

## Graph 3

$$
d\omega_0
$$

vs

$$
\boxed{\Delta\omega_{\rm motor}}
$$

これがV58の最重要グラフ。

---

## Graph 4

$$
d\omega_0
$$

vs

$$
Q_{50}
$$

---

## Graph 5

$$
d\omega_0
$$

vs

$$
\frac{\Delta\omega_{\rm motor}}{Q_{50}}
$$

---

## Graph 6

PROBEとCOASTのWheel speed time series代表例。

* negative aligned
* zero
* positive aligned

を表示する。

---

# 17. 反復数

各速度領域について最低限、

```text
PROBE × 5 valid
COAST × 5 valid
```

程度を目標とする。

5速度領域なら最低50 valid event程度。

可能なら2 Run以上へ分ける。

1 Run内の傾向だけでは結論を出さない。

---

# 18. 方向を両方含める

同じaligned-speed値でも物理回転方向が異なるデータを含める。

これにより、

$$
d\omega
$$

だけで+/-方向を統合できるか確認する。

方向差が残る場合はdirectionを別変数として残す。

---

# 19. V58で答える質問

### Q1

V57bで見えた大きな

$$
d\omega_0
\rightarrow d\Delta\omega
$$

関係は、単なる自然減速で説明できるか。

### Q2

自然減速を引いても、

$$
\Delta\omega_{\rm motor}
$$

は初期Wheel速度によって変化するか。

### Q3

$$
Q_{50}
$$

がほぼ一定でも、

$$
\Delta\omega_{\rm motor}
$$

は変化するか。

### Q4

$$
\Delta\omega_{\rm motor}/Q_{50}
$$

はWheel速度依存か。

### Q5

Roller485のcurrent readbackは、Wheelへ実際に作用した入力を十分表しているか。

---

# 20. V58の判定

## Case A：coast補正後は速度依存が消える

$$
\Delta\omega_{\rm motor}
$$

が初期速度によらずほぼ一定。

この場合、

**V57/V57bで見えた大きな速度応答差は主に自然減速によるもの**

と判断する。

Wheel速度をQ_availableモデルへ入れる根拠は弱くなる。

---

## Case B：coast補正後も速度依存が強く残る

$$
d\omega_0
$$

が正になるほど、

$$
\Delta\omega_{\rm motor}
$$

が系統的に低下。

しかも、

$$
Q_{50}
$$

はほぼ一定。

この場合、

**current readback上のQと、実際のモータ作用が一致していない可能性**

が強くなる。

次は外部current測定またはトルク相当量の独立測定へ進む。

---

## Case C：Q50自体も速度依存になる

より厳密な対照実験で、

$$
Q_{50}
$$

にも再現可能な速度依存が現れる。

この場合、

Wheel速度 → current/Q生成能力

という経路も再評価する。

---

# 21. V58ではやらないこと

* Q_availableモデル作成
* energy control変更
* Q_target変更
* currentモデル補正
* Q threshold補正
* Wheel speedフィードバック
* 外部currentセンサ導入

V58ではまず、

$$
\boxed{
\text{Wheel速度による観測上の機械応答差から、
自然減速分を直接取り除く}
}
$$

ことだけを行う。
