#ifndef UTILS_H
#define UTILS_H

#include <cstdint>
#include <vector>

#include "iir/iir.h"

void LockMemory();

void ConfigureRealtime(const uint8_t realtime);

class LowPassFilter {
public:
	LowPassFilter(int order, float cutoff_freq, float sampling_period_s);

	float iterate_filter(float sample);
	void flush_buffers();
private :
	std::vector<double> lpf_dcof_;
  std::vector<int> lpf_ccof_;
  double lpf_sf_;
  int lpf_order_ = 3;
  float lpf_fc_ = 40;
  std::vector<float> fib_;
  std::vector<float> fob_;

	float T_s_ = 0.001;
};

#endif