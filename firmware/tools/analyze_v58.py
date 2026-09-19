"""V58 paired probe / Current Mode zero-coast observational analysis.

Raw Q50 uses exact event-relative boundaries. Speed differences use actual
readback completion times, NOT a claim of exact sensor latency or pure torque.
"""
import argparse, copy, hashlib, json, math
from pathlib import Path
import numpy as np
from analyze_v57 import converter, rel, points_at, integral, read_current_csv, write_csv, plt
from analyze_v57b import analyze_trials as legacy_quality, finite, correlation, regression, SPEED, CURRENT, VOLTAGE, CENTERS

CALIPERS={SPEED:20., CURRENT:.1, VOLTAGE:.05, 'speed_observation_dt_us':500.}
DELTA='aligned_speed_delta_50_rpm'

def region(x):
    if not finite(x):return None
    if abs(x)<=20:return 0
    c=min((-300,-150,150,300),key=lambda c:(abs(c-x),abs(c)))
    return c if abs(c-x)<=75 else None

def analyze_trials(trials,current,n,source):
    # Reuse V57b's independently tested current/baseline checks only through
    # a temporary adapter. Output always restores genuine COAST zero-write fields.
    adapted=[dict(t,nonzero_write_end_us=t.get('event_start_us',0),valid_probe=t.get('valid_event',False)) for t in trials]
    rows,_=legacy_quality(adapted,current,n,source)
    for t,r in zip(trials,rows):
        for key in ('nonzero_write_begin_us','nonzero_write_end_us'):r[key]=t.get(key,0)
        r.pop('valid_probe',None)
        r['role']='COAST' if t.get('is_coast') else 'PROBE'
        r['actual_region_rpm']=region(r[SPEED]);errors=set(filter(None,r['quality_reasons'].split(';')))-{'region'}
        if r['actual_region_rpm'] is None or r['actual_region_rpm']!=t.get('requested_aligned_speed_rpm'):errors.add('region')
        start=t.get('event_start_us') or 0;end=t.get('speed_50ms_time_us') or 0;before=t.get('rw_speed_timestamp_us') or 0
        dt=rel(end,before);lag=rel(end,start);trace=t.get('speed_trace',[]);d=t.get('probe_direction',0)
        speed=t.get('speed_50ms_rpm');initial=t.get('rw_speed_before_probe_rpm')
        delta=d*(speed-initial) if finite(speed) and finite(initial) else None
        checks={'direction':d in (-1,1),'speed50':bool(t.get('speed_50ms_valid')) and 50000<=lag<=52000,
          'dt':dt>0 and dt==t.get('speed_observation_dt_us'),
          'trace':t.get('trace_count')==6 and len(trace)==6 and trace[0][0]==before and trace[-1][0]==end and all(len(v)==2 and finite(v[1]) for v in trace),
          'delta':finite(delta) and finite(t.get(DELTA)) and abs(delta-t[DELTA])<.001,
          'event':bool(start) and t.get('event_write_end_us')==start and rel(start,t.get('event_write_begin_us',start))>0,
          'role_event':(t.get('nonzero_write_end_us',0)==0 and t.get('nonzero_write_begin_us',0)==0 and t.get('coast_event_start_us')==start) if t.get('is_coast') else (t.get('nonzero_write_end_us')==start and t.get('coast_event_start_us',0)==0),
          'active_current':t.get('active_current_failures',1)==0 and t.get('active_current_max_gap_us',99999)<=3333}
        if checks['trace']:
            checks['trace_values']=abs(trace[0][1]-initial)<.001 and abs(trace[-1][1]-speed)<.001 and all(rel(b[0],a[0])>0 for a,b in zip(trace,trace[1:]))
        errors.update(k for k,v in checks.items() if not v)
        r[DELTA]=delta;r['speed50_lateness_us']=lag-50000
        r['speed_rate_rpm_s']=delta*1e6/dt if finite(delta) and dt>0 else None
        r['rate_normalized_50ms_delta_rpm']=r['speed_rate_rpm_s']*.05 if finite(r['speed_rate_rpm_s']) else None
        r['formal_quality_ok']=not errors;r['quality_reasons']=';'.join(sorted(errors))
        r['matched']=False;r['matching_reason']='QUALITY_REJECTED' if errors else 'NO_ADMISSIBLE_CONTROL'
    return rows

def analyze_one(path,out,n):
    dest=out/f'run_{n:02d}'/'converted';converter.convert(path,dest)
    m=json.loads((dest/'metadata.json').read_text(encoding='utf-8-sig'))
    if m.get('format')!='rwlog_paired_probe_v58' or not m.get('v58_metadata_complete') or m.get('metadata_error'):raise ValueError('Expected complete V58 metadata')
    p=m['paired_probe_v58'];cols=p.get('trial_columns');trials=p['trials']
    if cols:
        if any(len(t)!=len(cols) for t in trials):raise ValueError('Trial column count mismatch')
        trials=[dict(zip(cols,t)) for t in trials]
    rows=analyze_trials(trials,read_current_csv(dest/'fresh_current_samples.csv'),n,path.name)
    # Storage overflow is a run-level provenance defect, not silently ignored.
    q=m.get('q_observer_v58',{});w=m.get('wheel_observer_v58',{})
    if any([q.get('sample_overflow',0),q.get('pulse_overflow',0),w.get('overflow',0),p.get('voltage_overflow',0)]):
        for r in rows:r['formal_quality_ok']=False;r['quality_reasons']+=';storage_overflow'
    return rows,dict(file=str(path.resolve()),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),firmware_revision=m.get('firmware_revision'),run_id=json.loads((dest/'header.json').read_text())['run_id'],synthetic=bool(m.get('synthetic')),protocol=p)

def match(rows,calipers=CALIPERS):
    good=[r for r in rows if r['formal_quality_ok']];probes=[r for r in good if r['role']=='PROBE'];coasts=[r for r in good if r['role']=='COAST']
    for r in good:
        pool=[c for c in good if c['role']!=r['role'] and all(c[k]==r[k] for k in ('run_number','probe_direction','actual_region_rpm'))]
        r['same_group_control_count']=len(pool)
        if pool:
            near=min(pool,key=lambda c:sum(((r[k]-c[k])/calipers[k])**2 for k in calipers))
            r['nearest_opposite_role_id']=near['id']
            for k in calipers:r['nearest_difference_'+k]=r[k]-near[k]
    edges=[];candidates={id(r):0 for r in good};paired=[];used=set()
    for p in probes:
      for c in coasts:
        if any(p[k]!=c[k] for k in ('run_number','probe_direction','actual_region_rpm')):continue
        diffs={k:p[k]-c[k] for k in calipers}
        if any(abs(diffs[k])>calipers[k]+1e-9 for k in calipers):continue
        candidates[id(p)]+=1;candidates[id(c)]+=1
        edges.append((sum((diffs[k]/calipers[k])**2 for k in calipers),p['run_number'],p['id'],c['id'],p,c,diffs))
    for cost,_,__,___,p,c,diffs in sorted(edges,key=lambda x:x[:4]):
        if id(p) in used or id(c) in used:continue
        used.update((id(p),id(c)))
        for r in (p,c):r['matched']=True;r['matching_reason']='MATCHED'
        motor=p[DELTA]-c[DELTA];q=p['Q50_mA_s']
        paired.append(dict(run_number=p['run_number'],probe_direction=p['probe_direction'],actual_region_rpm=p['actual_region_rpm'],probe_id=p['id'],coast_id=c['id'],scheduled_pair_equal=p.get('pair_id')==c.get('pair_id'),cost=cost,
          **{SPEED:p[SPEED],CURRENT:p[CURRENT],VOLTAGE:p[VOLTAGE]},probe_delta_rpm=p[DELTA],coast_delta_rpm=c[DELTA],motor_delta_rpm=motor,Q50_mA_s=q,G_omegaQ_rpm_per_mA_s=motor/q if finite(q) and q>0 else None,
          probe_dt_us=p['speed_observation_dt_us'],coast_dt_us=c['speed_observation_dt_us'],rate_normalized_motor_50ms_rpm=(p['speed_rate_rpm_s']-c['speed_rate_rpm_s'])*.05,
          **{'difference_'+k:v for k,v in diffs.items()}))
    for r in good:
        r['admissible_candidate_count']=candidates[id(r)]
        if not r['matched'] and candidates[id(r)]:r['matching_reason']='CONTROL_ALREADY_USED'
    return paired

def features(x):
    x=np.asarray(x,dtype=float);return np.column_stack((np.ones(len(x)),x,np.clip(x/75,-1,1)))

def coast_regression(rows):
    models=[];estimates=[];good=[r for r in rows if r['formal_quality_ok']]
    for n in sorted({r['run_number'] for r in good}):
      for d in (1,-1):
        coast=[r for r in good if r['run_number']==n and r['probe_direction']==d and r['role']=='COAST']
        model=dict(run_number=n,probe_direction=d,n=len(coast),status='INSUFFICIENT_SUPPORT',basis=['1','speed','clip(speed/75,-1,1)'],outcome='measured_speed_rate_rpm_s')
        models.append(model)
        if len(coast)<8 or len({r['actual_region_rpm'] for r in coast})<3:continue
        x=np.array([r[SPEED] for r in coast]);y=np.array([r['speed_rate_rpm_s'] for r in coast]);X=features(x)
        if np.linalg.matrix_rank(X)<3:model['status']='RANK_DEFICIENT';continue
        b=np.linalg.lstsq(X,y,rcond=None)[0];res=y-X@b
        model.update(status='OBSERVATIONAL_ONLY',coefficients=b.tolist(),speed_support=[float(min(x)),float(max(x))],rmse_rpm_s=float(np.sqrt(np.mean(res**2))))
        for p in [r for r in good if r['run_number']==n and r['probe_direction']==d and r['role']=='PROBE']:
            e=dict(run_number=n,probe_direction=d,probe_id=p['id'],**{SPEED:p[SPEED]},status='OUTSIDE_COAST_SUPPORT',motor_delta_rpm=None,G_omegaQ_rpm_per_mA_s=None)
            estimates.append(e)
            # No extrapolation, and require nearby observed current/voltage support.
            if not min(x)<=p[SPEED]<=max(x):continue
            if not any(abs(p[SPEED]-c[SPEED])<=75 and abs(p[CURRENT]-c[CURRENT])<=CALIPERS[CURRENT] and abs(p[VOLTAGE]-c[VOLTAGE])<=CALIPERS[VOLTAGE] for c in coast):continue
            predicted=float((features([p[SPEED]])@b)[0])*p['speed_observation_dt_us']/1e6
            motor=p[DELTA]-predicted;q=p['Q50_mA_s']
            e.update(status='OBSERVATIONAL_ONLY',coast_delta_predicted_rpm=predicted,motor_delta_rpm=motor,G_omegaQ_rpm_per_mA_s=motor/q if finite(q) and q>0 else None)
    return models,estimates

def graphs(out,rows,pairs,estimates):
    good=[r for r in rows if r['formal_quality_ok']]
    series=[('PROBE_DELTA',[(r,r[DELTA],'PROBE') for r in good if r['role']=='PROBE'],'Measured directed speed change (rpm)'),
      ('COAST_DELTA',[(r,r[DELTA],'COAST') for r in good if r['role']=='COAST'],'Measured directed speed change (rpm)'),
      ('MOTOR_DELTA',[(r,r['motor_delta_rpm'],'paired') for r in pairs]+[(r,r['motor_delta_rpm'],'coast regression') for r in estimates],'Coast-subtracted speed change (rpm)'),
      ('Q50',[(r,r['Q50_mA_s'],'PROBE') for r in good if r['role']=='PROBE'],'Q50 (mA s)'),
      ('G_OMEGA_Q',[(r,r['G_omegaQ_rpm_per_mA_s'],'paired') for r in pairs]+[(r,r['G_omegaQ_rpm_per_mA_s'],'coast regression') for r in estimates],'Speed change / Q50 (rpm / mA s)')]
    for index,(name,pts,ylabel) in enumerate(series,1):
        fig,ax=plt.subplots(figsize=(8,5));groups=sorted({(r['run_number'],r['probe_direction'],method) for r,y,method in pts if finite(y)})
        for n,d,m in groups:
            s=[(r[SPEED],y) for r,y,method in pts if finite(y) and (r['run_number'],r['probe_direction'],method)==(n,d,m)]
            ax.scatter(*zip(*s),marker='o' if d==1 else 'x',label=f'Run {n}, d={d}, {m}')
        ax.set(xlabel='Actual directed initial speed (rpm)',ylabel=ylabel);ax.grid(True)
        if groups:ax.legend(fontsize=7)
        fig.tight_layout();fig.savefig(out/f'GRAPH_0{index}_{name}.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(13,4))
    for ax,center in zip(axes,(-300,0,300)):
        examples=[p for p in pairs if p['actual_region_rpm']==center]
        if examples:
            p=examples[0]
            for role,key in [('PROBE','probe_id'),('COAST','coast_id')]:
                r=next(r for r in good if r['run_number']==p['run_number'] and r['id']==p[key])
                trace=r['speed_trace'];ax.plot([rel(t,r['event_start_us'])/1000 for t,v in trace],[r['probe_direction']*v for t,v in trace],'.-',label=role)
            ax.legend()
        else:ax.text(.1,.5,'No matched example',transform=ax.transAxes)
        ax.set(title=f'Initial region {center:+d} rpm',xlabel='Time from event (ms)',ylabel='Directed speed (rpm)');ax.grid(True)
    fig.tight_layout();fig.savefig(out/'GRAPH_06_SPEED_TRACES.png',dpi=150);plt.close(fig)

def run(paths,out):
    out.mkdir(parents=True,exist_ok=True);rows=[];manifests=[];seen=set()
    for n,path in enumerate(paths,1):
        sha=hashlib.sha256(path.read_bytes()).hexdigest()
        if sha in seen:raise ValueError('Duplicate input cannot count as another run')
        seen.add(sha);rr,mm=analyze_one(path,out,n);rows+=rr;manifests.append(mm)
    pairs=match(rows);models,estimates=coast_regression(rows);good=[r for r in rows if r['formal_quality_ok']]
    coverage=[dict(run_number=n,region=c,probe_direction=d,role=role,valid_count=sum(r['run_number']==n and r['actual_region_rpm']==c and r['probe_direction']==d and r['role']==role for r in good)) for n in range(1,len(paths)+1) for c in CENTERS for d in (1,-1) for role in ('PROBE','COAST')]
    pair_coverage=[dict(run_number=n,region=c,probe_direction=d,matched_count=sum(p['run_number']==n and p['actual_region_rpm']==c and p['probe_direction']==d for p in pairs)) for n in range(1,len(paths)+1) for c in CENTERS for d in (1,-1)]
    common=[dict(p,regression_motor_delta_rpm=e['motor_delta_rpm'],method_difference_rpm=p['motor_delta_rpm']-e['motor_delta_rpm']) for p in pairs for e in estimates if e['run_number']==p['run_number'] and e['probe_id']==p['probe_id'] and finite(e['motor_delta_rpm'])]
    diagnostics={}
    for n in range(1,len(paths)+1):
      for d in (1,-1):
        probe=[r for r in good if r['run_number']==n and r['probe_direction']==d and r['role']=='PROBE']
        paired=[r for r in pairs if r['run_number']==n and r['probe_direction']==d]
        reg=[r for r in estimates if r['run_number']==n and r['probe_direction']==d]
        diagnostics[f'run_{n}_d_{d}']=dict(speed_precurrent_r=correlation(probe,SPEED,CURRENT),Q50_speed=regression(probe,'Q50_mA_s',[SPEED]),Q50_adjusted=regression(probe,'Q50_mA_s',[SPEED,CURRENT,VOLTAGE]),paired_motor_speed=regression(paired,'motor_delta_rpm',[SPEED]),regression_motor_speed=regression(reg,'motor_delta_rpm',[SPEED]))
    summary=dict(schema='v58_paired_probe_coast_analysis_v1',trial_count=len(rows),formal_quality_count=len(good),matched_pair_count=len(pairs),runs=manifests,calipers=CALIPERS,matching='same run/direction/region; global sorted scaled squared distance greedy one-to-one, no auto relaxation; scheduled pair is not proof of state matching',coverage=coverage,pair_coverage=pair_coverage,all_run_cells_at_least_five=bool(coverage) and all(c['valid_count']>=5 for c in coverage),all_run_pair_cells_at_least_five=bool(pair_coverage) and all(c['matched_count']>=5 for c in pair_coverage),at_least_two_hardware_runs=len(paths)>=2 and not any(m['synthetic'] for m in manifests),coast_models=models,diagnostics=diagnostics,decision='REVIEW_REQUIRED: no automatic Case A/B/C or control adoption',timing='Q50 exact 0..50ms current integral; speed endpoint at 50..52ms and pre-event read; primary difference retains actual dt with 500us pair caliper; rate-normalized 50ms is an approximation; regression fits observed coast rate and scales by probe dt',interpretation='Current Mode zero coast is not an unpowered motor. G is an observational response ratio, not a torque constant. Residual current and voltage can confound results; review paired and regression disagreement and timing differences.')
    write_csv(out/'V58_TRIALS.csv',rows);write_csv(out/'V58_MATCHED_PAIRS.csv',pairs);write_csv(out/'V58_COAST_REGRESSION.csv',estimates);write_csv(out/'V58_METHOD_COMPARISON.csv',common);write_csv(out/'V58_COVERAGE.csv',coverage);write_csv(out/'V58_PAIR_COVERAGE.csv',pair_coverage)
    traces=[dict(run_number=r['run_number'],id=r['id'],role=r['role'],time_us=t,event_relative_us=rel(t,r.get('event_start_us',0)),rpm=v,aligned_rpm=r['probe_direction']*v) for r in rows for t,v in r.get('speed_trace',[]) if finite(v)]
    write_csv(out/'V58_SPEED_TRACES.csv',traces);graphs(out,rows,pairs,estimates)
    (out/'V58_SUMMARY.json').write_text(json.dumps(summary,indent=2,allow_nan=False),encoding='utf-8');return summary

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('rwlog',nargs='+',type=Path);p.add_argument('--out',required=True,type=Path);a=p.parse_args();s=run(a.rwlog,a.out);print(json.dumps({k:s[k] for k in ('trial_count','formal_quality_count','matched_pair_count','decision')}))
