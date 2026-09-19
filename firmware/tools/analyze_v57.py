"""Offline analysis for V57 fixed 300 mA / 60 ms speed-capability runs.

This program never proposes a control threshold. It preserves the fixed-pulse
experiment, reports timing/data quality, and writes all requested regressions
as observational diagnostics.
"""
import argparse, bisect, csv, json, math, shutil
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import convert_rwlog_to_csv as converter

GAP_US = 3333
PROBE_POINTS_MS = (5, 10, 20, 30, 40, 50, 60)

def rel(t, origin): return (int(t)-int(origin)+2**31) % 2**32-2**31

def write_csv(path, rows):
    fields = list(dict.fromkeys(k for r in rows for k in r)) if rows else ["no_rows"]
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader(); w.writerows(rows)

def points_at(raw, origin, direction):
    out=[]
    for p in raw:
        t=rel(p["time_us"],origin)
        if -GAP_US <= t <= 70000:
            out.append((t, direction*p["current_mA"], bool(int(p["valid"]))))
    return sorted(out)

def value_at(points, t):
    times=[p[0] for p in points]; i=bisect.bisect_left(times,t)
    if i < len(points) and points[i][0] == t: return points[i][1] if points[i][2] else None
    if not 0 < i < len(points): return None
    a,b=points[i-1],points[i]
    if not a[2] or not b[2] or b[0]-a[0] > GAP_US: return None
    return a[1]+(b[1]-a[1])*(t-a[0])/(b[0]-a[0])

def integral(points, start, end):
    if end <= start: return None
    a=value_at(points,start); b=value_at(points,end)
    if a is None or b is None: return None
    grid=[(start,a,True)]+[(t,v,ok) for t,v,ok in points if start < t < end]+[(end,b,True)]
    q=0.0
    for x,y in zip(grid,grid[1:]):
        if not x[2] or not y[2] or y[0]-x[0] > GAP_US: return None
        q += .5*(x[1]+y[1])*(y[0]-x[0])*1e-6
    return q

def current_quality(points,start,end):
    inside=[p for p in points if start <= p[0] <= end]
    valid=[p for p in inside if p[2]]
    gaps=[b[0]-a[0] for a,b in zip(valid,valid[1:])]
    return dict(current_sample_count=len(inside), current_read_failures=sum(not p[2] for p in inside),
                current_max_gap_us=max(gaps) if gaps else None,
                current_gap_gt_3333_count=sum(g>GAP_US for g in gaps),
                current_timing_ok=bool(value_at(points,start) is not None and value_at(points,end) is not None and gaps and max(gaps)<=GAP_US and not any(not p[2] for p in inside)))

def read_current_csv(path):
    with path.open(encoding="utf-8-sig",newline="") as f:
        return [{"time_us":int(x["time_us"]),"current_mA":float(x["current_mA"]) if x["current_mA"] else math.nan,"valid":int(x["valid"])} for x in csv.DictReader(f)]

def analyze_one(raw_file, out, run_number):
    converted=out/f"run_{run_number:02d}"/"converted"; converter.convert(raw_file,converted)
    meta=json.loads((converted/"metadata.json").read_text(encoding="utf-8-sig"))
    if meta.get("metadata_error"):
        raise ValueError(f"{raw_file}: V57 metadata error: {meta['metadata_error']}")
    if meta.get("format") != "rwlog_fixed_probe_v57": raise ValueError(f"{raw_file}: not V57 RWLOG")
    if not meta.get("v57_metadata_complete"):
        raise ValueError(f"{raw_file}: V57 metadata is incomplete")
    protocol=meta.get("fixed_probe_v57",{}); current=read_current_csv(converted/"fresh_current_samples.csv")
    raw_trials=protocol.get("trials",[])
    columns=protocol.get("trial_columns")
    trials=[dict(zip(columns, t)) if columns and isinstance(t, list) else t for t in raw_trials]
    rows=[]; traces=[]
    for t in trials:
        direction=int(t.get("probe_direction") or 0); start=t.get("nonzero_write_end_us") or 0; zero_begin=t.get("zero_write_begin_us") or 0
        pts=points_at(current,start,direction) if start else []
        row={"source_file":raw_file.name,"run_number":run_number,"trial_id":t.get("id"),"trial_index":t.get("id"),
             "condition_repeat":t.get("condition_repeat"),"result":t.get("result"),"target_speed_rpm":t.get("target_speed_rpm"),
             "probe_direction":direction,"rw_speed_before_probe_rpm":t.get("rw_speed_before_probe_rpm"),
             "aligned_speed_before_probe_rpm":None,"rw_speed_after_probe_rpm":t.get("rw_speed_after_probe_rpm"),
             "aligned_speed_delta_rpm":t.get("aligned_speed_delta_rpm"),"baseline_current_mA":t.get("baseline_current_mA"),
             "current_before_probe_mA":t.get("current_before_probe_mA"),"bus_voltage_before_probe_V":(t.get("bus_voltage_before_probe_mV") or 0)/1000,
             "transfer_ready":t.get("transfer_ready"),"transfer_wait_ms":rel(t.get("transfer_ready_us") or 0,t.get("transfer_begin_us") or 0)/1000 if t.get("transfer_ready_us") else None,
             "speed_age_at_nonzero_end_us":t.get("rw_speed_age_at_nonzero_end_us"),"speed_fresh_at_probe":t.get("speed_fresh_at_probe"),
             "nonzero_write_begin_us":t.get("nonzero_write_begin_us"),"nonzero_write_end_us":start,"zero_write_begin_us":zero_begin,"zero_write_end_us":t.get("zero_write_end_us"),
             "planned_probe_us":t.get("planned_probe_us"),"actual_probe_width_us":t.get("actual_probe_width_us"),"zero_deadline_margin_us":t.get("zero_deadline_margin_us"),"zero_deadline_met":t.get("zero_deadline_met"),
             "q_probe_60ms_on_device_mA_s":t.get("q_probe_60ms_on_device_mA_s")}
        if row["rw_speed_before_probe_rpm"] is not None: row["aligned_speed_before_probe_rpm"]=direction*float(row["rw_speed_before_probe_rpm"])
        for ms in PROBE_POINTS_MS: row[f"current_{ms}ms_mA"]=value_at(pts,ms*1000)
        row["Q_probe_60ms_mA_s"]=integral(pts,0,rel(zero_begin,start)) if zero_begin else None
        row.update(current_quality(pts,0,rel(zero_begin,start)) if zero_begin else {})
        row["formal_quality_ok"]=bool(row["result"]=="COMPLETE" and row["transfer_ready"] and row["speed_fresh_at_probe"] and row["zero_deadline_met"] and row.get("current_timing_ok"))
        rows.append(row); traces.append((row,pts))
    return rows,traces,meta

def ols(rows,x,y):
    a=[(float(r[x]),float(r[y])) for r in rows if r.get(x) is not None and r.get(y) is not None and math.isfinite(float(r[x])) and math.isfinite(float(r[y]))]
    if len(a)<3:return {"n":len(a),"status":"INSUFFICIENT_DATA"}
    xx=np.array([p[0] for p in a]); yy=np.array([p[1] for p in a]); slope,intercept=np.polyfit(xx,yy,1); pred=slope*xx+intercept
    return {"n":len(a),"intercept":float(intercept),"slope_per_rpm":float(slope),"r_squared":float(1-np.sum((yy-pred)**2)/np.sum((yy-yy.mean())**2)) if np.ptp(yy)>0 else None,"status":"OBSERVATIONAL_ONLY"}

def graph(out,rows,traces):
    plt.rcParams.update({"font.size":9,"axes.grid":True})
    good=[r for r in rows if r.get("formal_quality_ok")]
    fig,ax=plt.subplots(figsize=(9,5));
    for r,p in traces:
        if r.get("formal_quality_ok"): ax.plot([x[0]/1000 for x in p],[x[1] if x[2] else math.nan for x in p],alpha=.35,label=None)
    ax.set(xlabel="Time from nonzero CURRENT write completion (ms)",ylabel="Direction-normalized current (mA)",xlim=(-2,65),title="1. Fixed-pulse raw current waveforms"); fig.tight_layout();fig.savefig(out/"GRAPH_01_RAW_CURRENT.png",dpi=160);plt.close(fig)
    specs=[("GRAPH_02_Q_VS_SPEED.png","Q_probe_60ms_mA_s","Q probe 60 ms (mA s)"),("GRAPH_03_I20_VS_SPEED.png","current_20ms_mA","Current at 20 ms (mA)"),("GRAPH_04_DELTA_SPEED.png","aligned_speed_delta_rpm","d*(speed_after-speed_before) (rpm)"),("GRAPH_05_VBUS_Q.png","Q_probe_60ms_mA_s","Q probe 60 ms (mA s)")]
    for name,y,label in specs:
        fig,ax=plt.subplots(figsize=(7,5)); x="bus_voltage_before_probe_V" if name=="GRAPH_05_VBUS_Q.png" else "aligned_speed_before_probe_rpm"
        for d,c in ((1,"#2878b5"),(-1,"#c24b42")):
            sub=[r for r in good if r.get("probe_direction")==d and r.get(x) is not None and r.get(y) is not None]
            ax.scatter([r[x] for r in sub],[r[y] for r in sub],label=f"d={d}",color=c)
        ax.set(xlabel=x,ylabel=label,title=name[6:-4]);ax.legend();fig.tight_layout();fig.savefig(out/name,dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,5)); ax.scatter(range(len(rows)),[r.get("current_max_gap_us") or math.nan for r in rows]);ax.axhline(GAP_US,color="r",ls="--");ax.set(xlabel="trial row",ylabel="max current gap (us)",title="6. Current timing audit");fig.tight_layout();fig.savefig(out/"GRAPH_06_TIMING.png",dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,5)); ax.scatter([r.get("aligned_speed_before_probe_rpm") for r in rows],[r.get("transfer_wait_ms") for r in rows]);ax.set(xlabel="aligned speed before probe (rpm)",ylabel="transfer wait (ms)",title="7. Transfer-ready wait by speed");fig.tight_layout();fig.savefig(out/"GRAPH_07_TRANSFER_WAIT.png",dpi=160);plt.close(fig)

def main():
    p=argparse.ArgumentParser();p.add_argument("rwlog",nargs="+",type=Path);p.add_argument("--out",type=Path,required=True);a=p.parse_args()
    a.out.mkdir(parents=True,exist_ok=True); all_rows=[]; all_traces=[]; manifests=[]
    for n,f in enumerate(a.rwlog,1):
        rows,traces,meta=analyze_one(f,a.out,n);all_rows+=rows;all_traces+=traces;manifests.append({"file":f.name,"format":meta.get("format"),"firmware_revision":meta.get("firmware_revision")})
    write_csv(a.out/"V57_FIXED_PROBE_TRIALS.csv",all_rows);graph(a.out,all_rows,all_traces)
    good=[r for r in all_rows if r.get("formal_quality_ok")]
    summary={"schema":"v57_fixed_probe_analysis_v1","runs":manifests,"trial_count":len(all_rows),"formal_quality_count":len(good),"fixed_protocol":{"current_mA":300,"planned_width_ms":60,"transfer_tolerance_mA":0.7,"transfer_continuous_ms":20,"transfer_limit_ms":500,"q_definition":"direction_normalized_raw_current_integral_nonzero_write_end_to_zero_write_begin"},"regressions":{"Q_vs_aligned_speed":ols(good,"aligned_speed_before_probe_rpm","Q_probe_60ms_mA_s"),"I20_vs_aligned_speed":ols(good,"aligned_speed_before_probe_rpm","current_20ms_mA"),"Q_vs_vbus":ols(good,"bus_voltage_before_probe_V","Q_probe_60ms_mA_s")},"decision":"OBSERVATIONAL_ONLY; do not adopt Case A/B/C or alter energy control without separate reviewed data."}
    (a.out/"V57_FIXED_PROBE_SUMMARY.json").write_text(json.dumps(summary,indent=2),encoding="utf-8")
    print(json.dumps({"trials":len(all_rows),"formal_quality":len(good),"out":str(a.out)},ensure_ascii=False))
if __name__=="__main__": main()