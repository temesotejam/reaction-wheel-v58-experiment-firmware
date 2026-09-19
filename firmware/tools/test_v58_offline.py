"""Analytical synthetic data; never evidence of hardware behavior."""
import copy,json,struct,zlib
from pathlib import Path
import analyze_v58 as a
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'analysis/v58_tests/offline'

def fixture(path,run):
    trials=[];events=[]
    for i,c in enumerate(a.CENTERS):
      for j,d in enumerate((1,-1)):
       for repeat in range(5):
        for coast in (False,True):
          n=len(trials);start=1000000+n*1000000;pre=(repeat-2)*.02;v=7400+repeat*30+j*10+run*5;dt=50500
          # Linear coast-rate model is nested in the prespecified regression.
          rate=-2*c;delta=rate*dt/1e6+(0 if coast else 10)
          before=start-200;end=start+50300
          trace=[[before,d*c]]+[[start+ms*1000+300,d*(c+delta*(ms*1000+500)/dt)] for ms in (10,20,30,40,50)]
          t=dict(id=n+1,condition_id=i*4+j*2+coast,pair_id=n//2+1,probe_direction=d,requested_aligned_speed_rpm=c,
            rw_speed_before_probe_rpm=d*c,current_before_probe_mA=d*pre,baseline_current_mA=0,
            baseline_current_mad_mA=.01,baseline_current_slope_mA_s=.01,baseline_current_spread_mA=.04,
            bus_voltage_before_probe_mV=v,nonzero_write_end_us=0 if coast else start,nonzero_write_begin_us=0 if coast else start-100,
            event_start_us=start,event_write_begin_us=start-100,event_write_end_us=start,coast_event_start_us=start if coast else 0,is_coast=coast,
            zero_write_begin_us=start+58500,zero_write_end_us=start+59000,actual_probe_width_us=59000,baseline_valid=True,transfer_ready=True,
            transfer_threshold_mA=.2,transfer_ready_begin_us=start-32000,transfer_ready_us=start-2000,
            fresh_current_valid=True,current_age_at_nonzero_end_us=500,speed_fresh_at_probe=True,
            rw_speed_timestamp_us=before,rw_speed_age_at_nonzero_end_us=200,zero_deadline_met=True,valid_event=True,result='COMPLETE',
            speed_50ms_valid=True,speed_50ms_time_us=end,speed_observation_dt_us=dt,speed_50ms_rpm=d*(c+delta),
            aligned_speed_delta_50_rpm=delta,speed_trace=trace,trace_count=6,active_current_failures=0,active_current_max_gap_us=1000)
          trials.append(t)
          for time in range(-1000,62000,1000):
              current=pre if coast else 50+time*.002
              events.append(struct.pack('<IIiIB3x',start+time,len(events)+1,round(d*current*100),100,1))
    # Array trial representation matches production, not just dictionaries.
    cols=list(trials[0]);meta=dict(format='rwlog_paired_probe_v58',v58_metadata_complete=True,synthetic=True,firmware_revision='SYNTHETIC_TEST_ONLY',q_observer_v58=dict(raw_per_mA=100),paired_probe_v58=dict(trial_columns=cols,trials=[[t[k] for k in cols] for t in trials]))
    m=json.dumps(meta).encode();h=dict.fromkeys(a.converter.HEADER_FIELDS,0);size=struct.calcsize(a.converter.HEADER_FORMAT);offset=size+len(m);body=b''.join(events)
    h.update(magic=b'RWLOG01\0',format_version=48,header_size=size,run_id=run,metadata_json_size=len(m),event_count=len(events),log_sample_size=struct.calcsize(a.converter.sample_format_for_version(48)),event_row_size=20,events_offset=offset,samples_offset=offset,summaries_offset=offset,crc_offset=offset+len(body),total_trials=100,flags=1)
    data=struct.pack(a.converter.HEADER_FORMAT,*[h[k] for k in a.converter.HEADER_FIELDS])+m+body;path.write_bytes(data+struct.pack('<I',zlib.crc32(data)&0xffffffff));return trials

if __name__=='__main__':
    OUT.mkdir(parents=True,exist_ok=True);paths=[OUT/f'synthetic_run_{n}.rwlog' for n in (1,2)]
    inputs=[fixture(p,n) for n,p in enumerate(paths,1)]
    summary=a.run(paths,OUT/'analysis');assert summary['formal_quality_count']==200,summary['formal_quality_count']
    assert summary['matched_pair_count']==100;assert not summary['at_least_two_hardware_runs']
    assert summary['all_run_cells_at_least_five'] and summary['all_run_pair_cells_at_least_five']
    assert len(list((OUT/'analysis').glob('GRAPH_*.png')))==6
    rows,m=a.analyze_one(paths[0],OUT/'verify',1);pairs=a.match(rows);models,estimates=a.coast_regression(rows)
    for r in rows:
        if r['role']=='PROBE':assert abs(r['Q50_mA_s']-5)<1e-9
        else:assert r['nonzero_write_end_us']==0
    assert all(abs(p['motor_delta_rpm']-10)<1e-9 for p in pairs)
    assert all(e['status']=='OBSERVATIONAL_ONLY' and abs(e['motor_delta_rpm']-10)<1e-9 for e in estimates)
    assert a.region(-225)==-150 and a.region(225)==150 and a.region(21) is None
    assert len({(p['run_number'],p['coast_id']) for p in pairs})==len(pairs)
    p=next(r for r in rows if r['role']=='PROBE');c=next(r for r in rows if r['role']=='COAST')
    for key,shift in [(a.SPEED,21),(a.CURRENT,.11),(a.VOLTAGE,.051),('speed_observation_dt_us',501),('run_number',1),('probe_direction',-2)]:
        pc=copy.deepcopy([p,c]);pc[1][key]+=shift;assert not a.match(pc),key
    rr=copy.deepcopy(rows);ext=copy.deepcopy(p);ext[a.SPEED]=400;ext['id']=201;rr.append(ext)
    _,es=a.coast_regression(rr);assert next(e for e in es if e['probe_id']==201)['status']=='OUTSIDE_COAST_SUPPORT'
    current=a.read_current_csv(OUT/'verify/run_01/converted/fresh_current_samples.csv');trial=inputs[0][0]
    for field,value in [('speed_50ms_valid',False),('speed_observation_dt_us',1),('event_write_end_us',0),('active_current_failures',1),('baseline_valid',False),('current_before_probe_mA',1),('actual_probe_width_us',50000),('trace_count',5),('nonzero_write_end_us',0)]:
        t=copy.deepcopy(trial);t[field]=value;assert not a.analyze_trials([t],current,1,'synthetic')[0]['formal_quality_ok'],field
    start=trial['event_start_us'];pts=a.points_at(current,start,1)
    assert a.integral([p for p in pts if not 18000<=p[0]<=23000],0,50000) is None
    assert a.integral([p for p in pts if p[0]<50000],0,50000) is None
    try:a.run([paths[0],paths[0]],OUT/'duplicate');raise AssertionError('duplicate accepted')
    except ValueError:pass
    original=paths[0].read_bytes();bad=OUT/'bad_crc.rwlog';bad.write_bytes(original[:-1]+bytes([original[-1]^1]))
    try:a.analyze_one(bad,OUT/'bad_crc',1);raise AssertionError('CRC accepted')
    except ValueError:pass
    result=dict(passed=True,synthetic_only=True,tests=['200 exact Q50 event integrals','100 unique pairs','known motor delta = 10 rpm for both methods','six plots','each caliper/run/direction mismatch rejected','no coast reuse','regression extrapolation rejected','speed trace/event/baseline/current gate rejections','current gap/missing boundary','CRC and duplicate rejection','225 rpm boundaries'])
    (OUT/'OFFLINE_TEST_RESULT.json').write_text(json.dumps(result,indent=2));print('V58 offline PASS')
