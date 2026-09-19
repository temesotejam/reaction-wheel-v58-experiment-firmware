"""V57b observational analysis. Exact Q50; no extrapolation or model adoption."""
import argparse, hashlib, json, math
from pathlib import Path
import numpy as np
from analyze_v57 import (converter, rel, points_at, value_at, integral, current_quality,
                         read_current_csv, write_csv, plt)
SPEED='aligned_speed_before_probe_rpm'; CURRENT='current_before_probe_aligned_mA'; VOLTAGE='bus_voltage_before_probe_V'
CENTERS=(-300,-150,0,150,300)
def finite(v): return isinstance(v,(int,float)) and math.isfinite(v)
def region(speed):
    if not finite(speed): return None
    if abs(speed)<=20:return 0
    center=min((-300,-150,150,300),key=lambda c:abs(speed-c))
    return center if abs(speed-center)<=75 else None

def analyze_trials(trials,current,run_number,source):
    rows=[];traces=[]
    for t in trials:
        r=dict(t,source_file=source,run_number=run_number)
        d=int(t.get('probe_direction') or 0);start=t.get('nonzero_write_end_us') or 0
        pts=points_at(current,start,d) if start else []
        speed=t.get('rw_speed_before_probe_rpm');r[SPEED]=d*speed if finite(speed) else None
        before=t.get('current_before_probe_mA');r[CURRENT]=d*before if finite(before) else None
        v=t.get('bus_voltage_before_probe_mV');r[VOLTAGE]=v/1000 if finite(v) else None
        r['actual_region_rpm']=region(r[SPEED]);r['Q50_mA_s']=integral(pts,0,50000)
        zero=t.get('zero_write_begin_us') or 0
        r['Q_actual_active_mA_s']=integral(pts,0,rel(zero,start)) if start and zero else None
        for ms in (5,10,20,30,40,50):r[f'I{ms}_mA']=value_at(pts,ms*1000)
        r.update(current_quality(pts,0,50000))
        checks={
          'completed':t.get('result')=='COMPLETE','device_valid':bool(t.get('valid_probe')),
          'baseline':bool(t.get('baseline_valid')),
          'baseline_MAD':finite(t.get('baseline_current_mad_mA')) and 0<=t['baseline_current_mad_mA']<=.100001,
          'baseline_spread':finite(t.get('baseline_current_spread_mA')) and 0<=t['baseline_current_spread_mA']<=.500001,
          'baseline_slope':finite(t.get('baseline_current_slope_mA_s')) and abs(t['baseline_current_slope_mA_s'])<=.200001,
          'transfer':bool(t.get('transfer_ready')),
          'fresh_current':bool(t.get('fresh_current_valid')) and 0<=t.get('current_age_at_nonzero_end_us',99999)<=3333,
          'fresh_speed':bool(t.get('speed_fresh_at_probe')) and 0<=t.get('rw_speed_age_at_nonzero_end_us',99999)<=2000,
          'deadline':bool(t.get('zero_deadline_met')),
          'width':50000<t.get('actual_probe_width_us',0)<=60000,
          'Q50':r['Q50_mA_s'] is not None and r['current_timing_ok'],
          'region':r['actual_region_rpm'] is not None and r['actual_region_rpm']==t.get('requested_aligned_speed_rpm'),
          'vbus':finite(r[VOLTAGE]) and 0<r[VOLTAGE]<100,
        }
        baseline=t.get('baseline_current_mA');threshold=t.get('transfer_threshold_mA')
        checks['residual']=finite(before) and finite(baseline) and finite(threshold) and 0.199999<=threshold<=.300001 and abs(before-baseline)<=threshold+1e-6
        checks['transfer_duration']=bool(t.get('transfer_ready_begin_us') and t.get('transfer_ready_us') and rel(t['transfer_ready_us'],t['transfer_ready_begin_us'])>=30000)
        r['formal_quality_ok']=all(checks.values());r['quality_reasons']=';'.join(k for k,v in checks.items() if not v)
        r['baseline_adjusted_aligned_precurrent_mA']=d*(before-baseline) if finite(before) and finite(baseline) else None
        rows.append(r);traces.append((r,pts))
    return rows,traces

def analyze_one(path,out,n):
    converted=out/f'run_{n:02d}'/'converted';converter.convert(path,converted)
    m=json.loads((converted/'metadata.json').read_text(encoding='utf-8-sig'))
    if m.get('format')!='rwlog_fixed_probe_v57b' or not m.get('v57_metadata_complete') or m.get('metadata_error'):raise ValueError('Expected complete V57b metadata: '+str(path))
    p=m['fixed_probe_v57b'];cols=p.get('trial_columns')
    trials=[dict(zip(cols,t)) if isinstance(t,list) else t for t in p['trials']]
    rows,traces=analyze_trials(trials,read_current_csv(converted/'fresh_current_samples.csv'),n,path.name)
    return rows,traces,dict(file=str(path.resolve()),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),firmware_revision=m.get('firmware_revision'),run_id=json.loads((converted/'header.json').read_text())['run_id'],protocol=p,synthetic=bool(m.get('synthetic')))

def correlation(rows,x,y):
    a=np.array([(r[x],r[y]) for r in rows if finite(r.get(x)) and finite(r.get(y))])
    if len(a)<3 or min(np.ptp(a,axis=0))<1e-12:return None
    return float(np.corrcoef(a.T)[0,1])

def regression(rows,y,predictors):
    good=[r for r in rows if all(finite(r.get(k)) for k in [y]+predictors)]
    result=dict(n=len(good),predictors=predictors,outcome=y,status='INSUFFICIENT_DATA')
    if len(good)<=len(predictors)+1:return result
    X=np.array([[r[k] for k in predictors] for r in good]);Y=np.array([r[y] for r in good]);scale=X.std(axis=0)
    if any(scale<1e-12):return dict(result,status='CONSTANT_PREDICTOR')
    Z=np.column_stack((np.ones(len(X)),(X-X.mean(axis=0))/scale))
    if np.linalg.matrix_rank(Z)<Z.shape[1]:return dict(result,status='RANK_DEFICIENT')
    b=np.linalg.lstsq(Z,Y,rcond=None)[0];pred=Z@b;coeff=b[1:]/scale
    cond=float(np.linalg.cond(Z));r2=float(1-((Y-pred)**2).sum()/((Y-Y.mean())**2).sum()) if np.ptp(Y)>1e-12 else None
    return dict(result,status='OBSERVATIONAL_ONLY',intercept=float(b[0]-coeff@X.mean(axis=0)),coefficients=dict(zip(predictors,map(float,coeff))),r_squared=r2,standardized_condition_number=cond,ill_conditioned=cond>30)

def audit(rows):
    corr=correlation(rows,SPEED,CURRENT)
    models={y:{'simple':regression(rows,y,[SPEED]),'adjusted':regression(rows,y,[SPEED,CURRENT,VOLTAGE])} for y in ('Q50_mA_s','I20_mA','aligned_speed_delta_rpm')}
    for pair in models.values():
        a=pair['simple'].get('coefficients',{}).get(SPEED);b=pair['adjusted'].get('coefficients',{}).get(SPEED)
        pair['speed_sign_agrees']=None if a is None or b is None else bool(a*b>0)
    return dict(n=len(rows),speed_precurrent_r=corr,ideal_abs_r_below_0p3=None if corr is None else abs(corr)<.3,near_perfect_confound=corr is not None and abs(corr)>=.9,regressions=models)

def graphs(out,rows,traces):
    good=[r for r in rows if r['formal_quality_ok']];colors=dict(zip(CENTERS,plt.cm.viridis(np.linspace(0,1,5))))
    fig,ax=plt.subplots(figsize=(8,5));seen=set()
    for r,points in traces:
        if not r['formal_quality_ok']:continue
        c=r['actual_region_rpm'];label=f'{c:+d} rpm' if c not in seen else None;seen.add(c)
        ax.plot([p[0]/1000 for p in points],[p[1] if p[2] else math.nan for p in points],color=colors[c],alpha=.35,label=label)
    ax.set(xlabel='Time after nonzero write completion (ms)',ylabel='Directed raw current (mA)',xlim=(-2,60),title='Current waveforms by actual aligned speed region');ax.grid(True)
    if seen:ax.legend()
    fig.tight_layout();fig.savefig(out/'GRAPH_06_CURRENT_BY_REGION.png',dpi=160);plt.close(fig)
    for i,(y,label) in enumerate([('Q50_mA_s','Q50 (mA s)'),('I20_mA','I20 (mA)'),('I50_mA','I50 (mA)'),('aligned_speed_delta_rpm','Directed speed change (rpm)'),(CURRENT,'Directed pre-probe current (mA)')],1):
        fig,ax=plt.subplots(figsize=(7,5))
        for run in sorted(set(r['run_number'] for r in good)):
            for d,marker in ((1,'o'),(-1,'x')):
                s=[r for r in good if r['run_number']==run and r['probe_direction']==d and finite(r.get(y))]
                if s:ax.scatter([r[SPEED] for r in s],[r[y] for r in s],marker=marker,label=f'Run {run}, d={d}')
        ax.set(xlabel='Actual aligned pre-probe speed (rpm)',ylabel=label);ax.grid(True)
        if good:ax.legend()
        fig.tight_layout();fig.savefig(out/f'GRAPH_0{i}_{y.upper()}.png',dpi=160);plt.close(fig)

def run(paths,out):
    out.mkdir(parents=True,exist_ok=True);rows=[];traces=[];manifests=[];seen=set()
    for n,path in enumerate(paths,1):
        sha=hashlib.sha256(path.read_bytes()).hexdigest()
        if sha in seen:raise ValueError('Duplicate input cannot count as an independent run')
        seen.add(sha);rr,tt,mm=analyze_one(path,out,n);rows+=rr;traces+=tt;manifests.append(mm)
    good=[r for r in rows if r['formal_quality_ok']]
    coverage=[dict(run_number=n,actual_region_rpm=c,probe_direction=d,valid_count=sum(r['run_number']==n and r['actual_region_rpm']==c and r['probe_direction']==d for r in good)) for n in range(1,len(paths)+1) for c in CENTERS for d in (1,-1)]
    groups={'all':audit(good)}
    for n in range(1,len(paths)+1):groups[f'run_{n}']=audit([r for r in good if r['run_number']==n])
    for d in (1,-1):
        groups[f'direction_{d}']=audit([r for r in good if r['probe_direction']==d])
        for n in range(1,len(paths)+1):groups[f'run_{n}_direction_{d}']=audit([r for r in good if r['run_number']==n and r['probe_direction']==d])
    summary=dict(schema='v57b_q50_confound_audit_v1',trial_count=len(rows),formal_quality_count=len(good),runs=manifests,coverage=coverage,at_least_two_hardware_runs=len(paths)>=2 and not any(m['synthetic'] for m in manifests),all_run_cells_at_least_five=all(c['valid_count']>=5 for c in coverage),audits=groups,decision='REVIEW_REQUIRED; no automatic Case A/B/C or speed model adoption',confound_policy='abs(r)<0.3 is an ideal diagnostic; abs(r)>=0.9 blocks independent speed interpretation; neither is a motor safety threshold',primary_Q_definition='raw directed trapezoid from write completion to +50000us; adjacent interpolation only, gap <=3333us')
    if any(g['near_perfect_confound'] for g in groups.values()):summary['decision']='SPEED_EFFECT_NOT_IDENTIFIED_DUE_TO_CONFOUND; do not adopt speed model'
    write_csv(out/'V57B_TRIALS.csv',rows);write_csv(out/'V57B_COVERAGE.csv',coverage);graphs(out,rows,traces)
    (out/'V57B_SUMMARY.json').write_text(json.dumps(summary,indent=2,allow_nan=False),encoding='utf-8')
    return summary
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('rwlog',type=Path,nargs='+');p.add_argument('--out',required=True,type=Path);a=p.parse_args();s=run(a.rwlog,a.out);print(json.dumps({k:s[k] for k in ('trial_count','formal_quality_count','decision')}))
