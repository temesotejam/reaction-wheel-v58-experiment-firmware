#include "Wire.h"
#include "roller485_manager.h"
#include <assert.h>
#include <cmath>
#include <iostream>

uint32_t fake_us = 10000;
FakeWire Wire;

static void setup(Roller485Manager& r, bool full = false) {
  fake_us = 10000; Wire = FakeWire{}; Wire.speed_raw = 0; Wire.current_raw = 0;
  assert(r.begin()); r.qObserver().reset(true); assert(r.beginFixedProbe(full, 1)); assert(!r.qObserver().rawSampleRecording()); Wire.writes.clear();
}

static void tick(Roller485Manager& r, bool transfer_ready = true) {
  auto& v = r.qObserver().v57;
  if (auto* t = v.current()) {
    if (v.phase == FixedProbeV57::PREPARING) {
      const bool speed_live = Wire.regs[0x00] == 1 && Wire.regs[0x01] == 1 && Wire.active_speed_target_raw != 0;
      Wire.speed_raw = speed_live ? Wire.active_speed_target_raw : 0;
      Wire.current_raw = speed_live ? (Wire.active_speed_target_raw > 0 ? 12000 : -12000) : 0;
    } else if (v.phase == FixedProbeV57::TRANSFER_WAIT) {
      Wire.speed_raw = t->target_speed_rpm * 80; Wire.current_raw = transfer_ready ? 0 : 300;
    } else if (v.phase == FixedProbeV57::PROBING) {
      assert(Wire.regs[0x01]==3 && Wire.regs[0xB0]==(t->is_coast?0:t->probe_direction*30000));
      Wire.current_raw = t->is_coast ? 0 : t->probe_direction * 30000;
    } else { Wire.speed_raw = 0; Wire.current_raw = 0; }
  }
  fake_us += 500;
  r.updatePulseCurrent(); r.updateFullStatus(); r.updateFixedProbe();
}

static void run_complete(bool full) {
  Roller485Manager r; setup(r, full); uint32_t guard = 0;
  while (r.qObserver().v57.active()) { tick(r); assert(++guard < 300000); }
  const auto& v = r.qObserver().v57;
  if (!(v.finished && !v.aborted && v.reason == FixedProbeV57::COMPLETE)) { std::cerr << "run failed full=" << full << " reason=" << FixedProbeV57::name(v.reason) << " samples=" << r.qObserver().sample_count << " overflow=" << r.qObserver().sample_overflow << " voltage_overflow=" << v.voltage_overflow << " guard=" << guard << "\\n"; assert(false); }
  std::cout << "full="<<full<<" count="<<unsigned(v.trial_count)<<" valid="<<unsigned(v.valid_probe_count)<<" expected="<<unsigned(full?FixedProbeV57::FULL_TRIAL_COUNT:FixedProbeV57::SIMPLE_TRIAL_COUNT)<<"\n";
  for(unsigned k=0;k<v.trial_count;++k)if(!v.trials[k].valid_probe)std::cerr<<"invalid "<<k<<" "<<FixedProbeV57::name(v.trials[k].result)<<" trace="<<unsigned(v.trials[k].trace_count)<<" dt="<<v.trials[k].speed_50ms_time_us-v.trials[k].event_start_us<<"\n";
  assert(v.trial_count == (full ? FixedProbeV57::FULL_TRIAL_COUNT : FixedProbeV57::SIMPLE_TRIAL_COUNT));
  assert(v.valid_probe_count==(full?100:20));
  for (uint8_t i = 0; i < v.trial_count; ++i) {
    const auto& t = v.trials[i];
    assert(t.baseline_begin_us-t.start_us>=FixedProbeV57::MIN_SETTLE_US);
    assert(t.baseline_end_us-t.baseline_begin_us>=FixedProbeBaseline::WINDOW_US);
    assert(t.baseline_valid && t.transfer_threshold_mA<=.300001);
    assert(t.result == FixedProbeV57::COMPLETE && t.transfer_ready && t.transition_ok);
    assert(t.baseline_current_samples >= FixedProbeV57::BASELINE_MIN_CURRENT_SAMPLES && std::isfinite(t.baseline_current_mA));
    assert(t.transfer_ready_us - t.transfer_ready_begin_us >= FixedProbeV57::TRANSFER_CONTINUOUS_US);
    assert(t.speed_fresh_at_probe && t.rw_speed_age_at_nonzero_end_us <= FixedProbeV57::MAX_SPEED_AGE_US);
    assert(t.event_write_end_us > t.event_write_begin_us);
    assert(t.speed_50ms_valid && t.trace_count==6 && t.speed_observation_dt_us>=50000);
    if(t.is_coast) assert(t.nonzero_write_end_us==0 && t.coast_event_start_us==t.event_start_us);
    else assert(t.nonzero_write_end_us==t.event_start_us);
    if(i%2) assert(v.trials[i-1].pair_id==t.pair_id && v.trials[i-1].is_coast!=t.is_coast);
    assert(t.actual_probe_width_us <= FixedProbeV57::PROBE_US && t.zero_deadline_met && t.zero_deadline_margin_us >= 0);
    assert(t.active_current_failures == 0 && t.active_current_max_gap_us <= QObserver::GAP_US);
    assert(std::isfinite(t.q_probe_60ms_on_device_mA_s) && (t.is_coast ? t.q_probe_60ms_on_device_mA_s==0 : t.q_probe_60ms_on_device_mA_s>0));
    assert(std::isfinite(t.rw_speed_after_probe_rpm));
  }
  if(full)for(unsigned i=20;i<v.trial_count;i+=2){const auto& t=v.trials[i];
    bool found=false;for(unsigned j=(i/20-1)*20;j<(i/20)*20;j+=2)if(v.trials[j].condition_id/2==t.condition_id/2){assert(v.trials[j].is_coast!=t.is_coast);found=true;}
    assert(found);}
  for (const auto& write : Wire.writes) if (write.reg == 0xB0 && write.value != 0) assert(std::abs(write.value) == 30000);
  assert(Wire.regs[0x00] == 0 && Wire.regs[0x01] == 3 && Wire.timeout_ms == 20);
  assert(!r.qObserver().sample_overflow && !r.qObserver().pulse_overflow && !r.qObserver().wheel.overflow);
  // Full V57 runs retain pulse windows, not all baseline/preparation reads.
  assert(r.qObserver().sample_count < (full ? 60000U : 12000U));
}


static void reach_probe(Roller485Manager& r) {
  uint32_t guard = 0;
  while (r.qObserver().v57.active() && r.qObserver().v57.phase != FixedProbeV57::PROBING) {
    tick(r); assert(++guard < 30000);
  }
  assert(r.qObserver().v57.phase == FixedProbeV57::PROBING);
}
static uint32_t approach_zero(Roller485Manager& r) {
  const auto* t = r.qObserver().v57.current();
  const uint32_t boundary = t->event_start_us + FixedProbeV57::PROBE_US - FixedProbeV57::ZERO_RESERVATION_US;
  Wire.current_raw = t->is_coast ? 0 : t->probe_direction * 30000;
  while (fake_us < boundary - 1000) {
    fake_us += 100; r.updatePulseCurrent(); r.updateFixedProbe();
    assert(r.qObserver().v57.active());
  }
  return boundary;
}
int main() {
  { Roller485Manager r; setup(r); reach_probe(r);
    const uint32_t boundary = approach_zero(r);
    const uint32_t reads = Wire.current_reads;
    const uint32_t sequence = r.telemetry().current_sequence;
    const uint8_t index = r.qObserver().v57.index;
    fake_us = boundary - 1;
    // First safety poll is just before reservation, second poll inside
    // readCurrentFresh is at reservation. No I2C failure is injected.
    r.updateFixedProbe();
    const auto& v = r.qObserver().v57;
    std::cout << "normal_deadline_transition"
              << " reason=" << FixedProbeV57::name(v.reason)
              << " aborted=" << v.aborted << " index=" << unsigned(v.index)
              << " current_reads_delta=" << Wire.current_reads-reads
              << " sequence_delta=" << r.telemetry().current_sequence-sequence
              << " failures=" << r.telemetry().current_read_failure_count
              << " previous_result=" << FixedProbeV57::name(v.trials[index].result)
              << " deadline_met=" << v.trials[index].zero_deadline_met << "\n";
    assert(v.index == index+1);
    assert(v.trials[index].result == FixedProbeV57::COMPLETE && v.trials[index].zero_deadline_met);
    assert(Wire.current_reads == reads && r.telemetry().current_sequence == sequence);
    assert(r.telemetry().current_read_failure_count == 0);
    assert(v.active() && !v.aborted);
    assert(Wire.regs[0xB0] == 0);
    tick(r); assert(v.active());
  }
  {FixedProbeV57 v;v.begin(true,1,1);assert(v.aborted && v.reason==FixedProbeV57::STORAGE_LIMIT);}
  {Roller485Manager r;setup(r);reach_probe(r);auto* t=r.qObserver().v57.current();
    while(fake_us-t->event_start_us<9000)tick(r);
    Wire.fail_speed_read=true;for(int k=0;k<5 && r.qObserver().v57.active();++k)tick(r);
    assert(r.qObserver().v57.reason==FixedProbeV57::SPEED_READ_FAILED && Wire.regs[0xB0]==0 && Wire.regs[0x00]==0);}
  { Roller485Manager r;setup(r);reach_probe(r);const auto index=r.qObserver().v57.index;
    Wire.speed_extra_duration=1900;unsigned guard=0;
    while(r.qObserver().v57.active() && r.qObserver().v57.index==index){tick(r);assert(++guard<1000);}
    const auto& v=r.qObserver().v57;const auto& t=v.trials[index];
    assert(v.active() && !v.aborted && t.result==FixedProbeV57::SPEED_OBSERVATION_INVALID);
    assert(t.zero_deadline_met && t.active_current_max_gap_us<=3333 && Wire.regs[0xB0]==0);
  }
  run_complete(false); run_complete(true);
  { Roller485Manager r; setup(r); Wire.fail_current_read_count=1; r.updateFixedProbe();
    assert(r.qObserver().v57.active() && r.telemetry().current_read_failure_count==1); }
  { Roller485Manager r; setup(r); Wire.fail_current_read_count=2; r.updateFixedProbe();
    assert(r.qObserver().v57.aborted && r.qObserver().v57.reason==FixedProbeV57::CURRENT_READ_FAILED);
    assert(Wire.regs[0x00]==0 && Wire.regs[0xB0]==0); }
  { Roller485Manager r; setup(r); reach_probe(r); Wire.fail_current_read_count=1; r.updateFixedProbe();
    assert(r.qObserver().v57.aborted && r.qObserver().v57.reason==FixedProbeV57::CURRENT_READ_FAILED);
    assert(Wire.regs[0x00]==0 && Wire.regs[0xB0]==0); }
  { Roller485Manager r; setup(r); reach_probe(r); fake_us=approach_zero(r); r.serviceMeasuredQStop();
    assert(r.qObserver().v57.active() && r.qObserver().observingPost());
    Wire.fail_current_read_count=1; r.updateFixedProbe();
    assert(r.qObserver().v57.aborted && r.qObserver().v57.reason==FixedProbeV57::CURRENT_READ_FAILED);
    assert(Wire.regs[0x00]==0 && Wire.regs[0xB0]==0); }
  { Roller485Manager r; setup(r); fake_us+=QObserver::GAP_US+1; r.serviceMeasuredQStop();
    assert(r.qObserver().v57.reason==FixedProbeV57::CURRENT_STALE && Wire.regs[0x00]==0); }
  { Roller485Manager r; setup(r); reach_probe(r); fake_us=approach_zero(r); Wire.fail_zero=true;
    r.serviceMeasuredQStop();
    assert(r.qObserver().v57.aborted && r.qObserver().v57.reason==FixedProbeV57::DRIVER_FAILURE);
    assert(Wire.regs[0x00]==0); }
  { Roller485Manager r; setup(r); uint32_t guard=0;
    while(r.qObserver().v57.active()) { tick(r,false); assert(++guard<150000); }
    assert(!r.qObserver().v57.aborted && r.qObserver().v57.reason==FixedProbeV57::COVERAGE_INCOMPLETE && r.qObserver().v57.failed_trial_count==40); }
  { Roller485Manager r;setup(r,true);uint32_t guard=0;
    while(r.qObserver().v57.active()){tick(r,false);assert(++guard<800000);}
    const auto& v=r.qObserver().v57;assert(!v.aborted&&v.trial_count==200&&v.valid_probe_count==0&&v.failed_trial_count==200);
    assert(v.reason==FixedProbeV57::COVERAGE_INCOMPLETE); }
  { Roller485Manager r;setup(r);uint32_t guard=0;
    while(r.qObserver().v57.phase!=FixedProbeV57::TRANSFER_WAIT){tick(r);assert(++guard<30000);}
    auto& v=r.qObserver().v57;auto* t=v.current();t->transfer_ready_begin_us=fake_us-30000;
    Wire.current_raw=100;fake_us+=500;r.updateFixedProbe();
    assert(v.phase==FixedProbeV57::TRANSFER_WAIT&&t->transfer_ready_begin_us==0&&Wire.regs[0xB0]==0);
  }
  { FixedProbeBaseline b;
    bool accepted=false;
    for(unsigned n=0;n<160;++n) { const float current=5.0f*expf(-float(n)*0.005f/0.07f); accepted=b.note(10000+n*5000,current,0); if(n<70) assert(!accepted); }
    assert(accepted && b.mad_mA<0.1f && fabsf(b.slope_mA_s)<=0.2f);
    b.clear();for(unsigned n=0;n<100;++n) assert(!b.note(10000+n*5000,n%2?1.0f:-1.0f,0));
    b.clear();for(unsigned n=0;n<100;++n) assert(!b.note(10000+n*5000,0,10));
  }
  { FixedProbeV57 a,b; a.trials=new FixedProbeV57::Trial[200];b.trials=new FixedProbeV57::Trial[200]; a.begin(true,10000,1);b.begin(true,10000,2);
    assert(a.order_seed!=b.order_seed);bool different=false;
    for(int i=0;i<10;++i) different=different||a.order[i]!=b.order[i];assert(different);
  }
  { Roller485Manager r;setup(r);uint32_t guard=0;
    while(r.qObserver().v57.phase!=FixedProbeV57::TRANSFER_WAIT) {tick(r);assert(++guard<30000);}
    auto& v=r.qObserver().v57;auto* t=v.current();t->requested_aligned_speed_rpm=150;t->target_speed_rpm=200;
    const auto c=t->condition_id;v.preparation_magnitude[c]=200;
    t->rw_speed_before_probe_rpm=0;r.qObserver().v57.failCurrentTrial(FixedProbeV57::PROBE_SPEED_REGION_MISSED,fake_us);
    assert(v.preparation_magnitude[c]==225 && v.active() && v.valid_probe_count==0);
  }
  { Roller485Manager r;setup(r);fake_us=r.qObserver().v57.start_us+FixedProbeV57::RUN_LIMIT_MS*1000UL;
    r.serviceMeasuredQStop();const auto& v=r.qObserver().v57;
    assert(v.finished&&!v.aborted&&v.reason==FixedProbeV57::COVERAGE_INCOMPLETE);
    assert(Wire.regs[0x00]==0&&Wire.regs[0xB0]==0); }
  std::cout << "V58 production: normal deadline transition, 20/100 paired event runs, retry, pulse/post failures, stale, zero-write failure, reachability PASS\n";
}
