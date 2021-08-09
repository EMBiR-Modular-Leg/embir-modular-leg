#include "utils.h"

#include <stdexcept>
#include <sys/mman.h>
#include <iostream>
#include <algorithm>

void LockMemory() {
  // We lock all memory so that we don't end up having to page in
  // something later which can take time.
  {
    const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
    if (r < 0) {
      throw std::runtime_error("Error locking memory");
    }
  }
}

void ConfigureRealtime(const uint8_t realtime) {
  {
    cpu_set_t cpuset = {};
    CPU_ZERO(&cpuset);
    CPU_SET(realtime, &cpuset);

    const int r = ::sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
    if (r < 0) {
      throw std::runtime_error("Error setting CPU affinity");
    }

    std::cout << "Affinity set to " << (int)realtime << "\n";
  }

  {
    struct sched_param params = {};
    params.sched_priority = 10;
    const int r = ::sched_setscheduler(0, SCHED_RR, &params);
    if (r < 0) {
      throw std::runtime_error("Error setting realtime scheduler");
    }
  }

  {
    const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
    if (r < 0) {
      throw std::runtime_error("Error locking memory");
    }
  }
}

LowPassFilter::LowPassFilter(int order, float cutoff_freq, float sampling_period_s) :
  lpf_order_(order), lpf_fc_(cutoff_freq), T_s_(sampling_period_s) {

  std::cout << "initializing filter with order " << lpf_order_ 
    << " and cutoff " << lpf_fc_ << "Hz..." << std::flush;
  fib_.resize(lpf_order_+1);
  fob_.resize(lpf_order_+1);
  lpf_dcof_ = dcof_bwlp(lpf_order_, 2*lpf_fc_*T_s_);
  lpf_ccof_ = ccof_bwlp(lpf_order_);
  lpf_sf_ = sf_bwlp(lpf_order_,  2*lpf_fc_*T_s_);
  std::cout << " done.\n";
  
  return;
}

float LowPassFilter::iterate_filter(float sample) {
  // rotate buffers one element to the right to put new data in
  std::rotate(fib_.rbegin(), fib_.rbegin()+1, fib_.rend());
  std::rotate(fob_.rbegin(), fob_.rbegin()+1, fob_.rend());
  fib_[0] = sample;
  fob_[0] = 0; // this term should cancel below 
  sample = 0;
  for (size_t ii = 0; ii < lpf_order_+1; ++ii) {
    sample += fib_[ii]*lpf_ccof_[ii]*lpf_sf_ - fob_[ii]*lpf_dcof_[ii];
  }
  fob_[0] = sample;

  return sample;
}

void LowPassFilter::flush_buffers() {
  for (auto& elem : fob_) elem = 0;
  for (auto& elem : fib_) elem = 0;
}
