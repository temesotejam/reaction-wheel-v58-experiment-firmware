#pragma once
#include "Arduino.h"
#include <vector>
struct FakeWire {
  struct Write { uint8_t reg; int32_t value; uint32_t time; };
  std::vector<Write> writes;
  std::vector<uint8_t> tx, rx;
  uint8_t selected = 0;
  int32_t vin_raw=750;
  int fail_write_reg=-1;
  int32_t regs[256]={};
  // Vendor has a readback shadow separate from the active speed PID target.
  int32_t active_speed_target_raw=0;
  bool ignore_mode_write=false, ignore_speed_write=false;
  uint32_t mode_reset_count=0;
  int32_t current_raw = 10000;
  int32_t speed_raw = -12345;
  uint32_t speed_reads = 0;
  bool fail_read = false, fail_zero = false, fail_speed_read = false, fail_mode = false;
  uint32_t fail_current_read_count = 0;
  uint32_t read_duration = 300, speed_extra_duration = 0;
  uint32_t write_duration = 100;
  uint32_t current_reads = 0;
  void begin(int,int) {}
  void setClock(uint32_t) {}
  uint32_t timeout_ms=20;
  void setTimeOut(uint32_t value) { timeout_ms=value; }
  void beginTransmission(uint8_t) { tx.clear(); }
  void write(uint8_t b) { tx.push_back(b); }
  void write(uint8_t* b, size_t n) { tx.insert(tx.end(),b,b+n); }
  uint8_t endTransmission(bool = true) {
    if (tx.empty()) return 0;
    selected=tx[0];
    if (tx.size()==1) return 0;
    int32_t v=0;
    memcpy(&v,tx.data()+1,tx.size()-1);
    writes.push_back({selected,v,fake_us}); fake_us+=write_duration;
    if(!((fail_zero && selected==0xB0 && v==0)||(fail_mode && selected==0x01) || selected==fail_write_reg)) {
      if (!((ignore_mode_write && selected==0x01) || (ignore_speed_write && selected==0x40))) {
        if (selected==0x01 && regs[selected]!=v) {
          active_speed_target_raw=0; ++mode_reset_count;
        }
        regs[selected]=v;
        if (selected==0x40) active_speed_target_raw=v;
      }
    }
    return ((fail_zero && selected==0xB0 && v==0) || (fail_mode && selected==0x01) || selected==fail_write_reg) ? 1 : 0;
  }
  uint8_t requestFrom(uint8_t,uint8_t n) {
    if (selected == 0xC0) ++current_reads;
    if (selected == 0x60) ++speed_reads;
    fake_us+=read_duration+(selected==0x60?speed_extra_duration:0);
    if (selected == 0xC0 && fail_current_read_count > 0) { --fail_current_read_count; return 0; }
    if (fail_read || (fail_speed_read && selected==0x60)) return 0;
    int32_t value=selected==0xC0 ? current_raw : selected==0x60 ? speed_raw : selected==0x34 ? vin_raw : regs[selected];
    rx.resize(n); memcpy(rx.data(),&value,n); return n;
  }
  int available() { return static_cast<int>(rx.size()); }
  int read() { auto b=rx.front(); rx.erase(rx.begin()); return b; }
};
extern FakeWire Wire;
