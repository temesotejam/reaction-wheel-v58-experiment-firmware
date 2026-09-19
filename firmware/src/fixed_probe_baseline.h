#pragma once
#include <algorithm>
#include <math.h>
#include <stdint.h>

// Only real readbacks are used. No residual-current model is subtracted.
struct FixedProbeBaseline {
  static constexpr unsigned N = 41;
  static constexpr uint32_t PERIOD_US = 5000, WINDOW_US = 200000;
  static constexpr float MAX_MAD_MA = 0.1f, MAX_SPREAD_MA = 0.5f;
  static constexpr float MAX_SLOPE_MA_S = 0.2f;
  struct Point { uint32_t time; float current, speed; };
  Point points[N];
  unsigned count = 0, next = 0;
  uint32_t last = 0, begin_us = 0, end_us = 0;
  float median_mA = NAN, mad_mA = NAN, slope_mA_s = NAN, spread_mA = NAN, speed_median = NAN;
  void clear() { count = next = 0; last = 0; }
  static float median(float* a, unsigned n) {
    std::sort(a, a+n);
    return n%2 ? a[n/2] : (a[n/2-1]+a[n/2])*0.5f;
  }
  bool note(uint32_t time, float current, float speed) {
    if (!isfinite(current) || !isfinite(speed) || fabsf(speed)>5.0f) { clear(); return false; }
    if (count && uint32_t(time-last)<PERIOD_US) return false;
    if (count && uint32_t(time-last)>2*PERIOD_US) clear();
    points[next] = {time,current,speed}; next=(next+1)%N;
    if (count<N) ++count;
    last=time;
    if(count<N) return false;
    begin_us=points[next].time; end_us=time;
    if(uint32_t(end_us-begin_us)<WINDOW_US) return false;
    float tail[N], dev[N], speeds[N];
    double sx=0, sy=0, sxx=0, sxy=0;
    float lo=points[next].current, hi=lo;
    unsigned nt=0;
    for(unsigned i=0;i<N;++i) {
      const auto& q=points[(next+i)%N];
      const double x=uint32_t(q.time-begin_us)*1e-6;
      sx+=x;sy+=q.current;sxx+=x*x;sxy+=x*q.current;
      lo=fminf(lo,q.current);hi=fmaxf(hi,q.current);speeds[i]=q.speed;
      if(i>=N/2) tail[nt++]=q.current;
    }
    median_mA=median(tail,nt);
    for(unsigned i=0;i<nt;++i) dev[i]=fabsf(tail[i]-median_mA);
    mad_mA=median(dev,nt);speed_median=median(speeds,N);spread_mA=hi-lo;
    const double denom=N*sxx-sx*sx;
    slope_mA_s=denom>0 ? (N*sxy-sx*sy)/denom : NAN;
    return isfinite(slope_mA_s) && mad_mA<=MAX_MAD_MA && spread_mA<=MAX_SPREAD_MA &&
           fabsf(slope_mA_s)<=MAX_SLOPE_MA_S;
  }
};
