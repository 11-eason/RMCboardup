#ifndef IMU_CALIBRATION_NOISE_HPP
#define IMU_CALIBRATION_NOISE_HPP

#include <cmath>
#include <cstdint>

// Startup-only statistics. Does not filter samples used by the attitude filter.
class ImuCalibrationNoise {
public:
  void reset() {*this=ImuCalibrationNoise{};}

  bool add(const float gyro[3], uint32_t now_ms, uint32_t block_ms, float max_mean_spread) {
    if (samples_==0) block_start_=now_ms;
    ++samples_; ++block_samples_;
    for (unsigned i=0;i<3;++i) {
      const float delta=gyro[i]-mean_[i];
      mean_[i]+=delta/static_cast<float>(samples_);
      m2_[i]+=delta*(gyro[i]-mean_[i]);
      block_sum_[i]+=gyro[i];
    }
    if (now_ms-block_start_>=block_ms && block_samples_>=8U) {
      for (unsigned i=0;i<3;++i) {
        const float average=block_sum_[i]/block_samples_;
        if (!have_block_) mean_min_[i]=mean_max_[i]=average;
        if (average<mean_min_[i]) mean_min_[i]=average;
        if (average>mean_max_[i]) mean_max_[i]=average;
        if (mean_max_[i]-mean_min_[i]>max_mean_spread) return false;
        block_sum_[i]=0.0f;
      }
      have_block_=true;
      block_samples_=0;
      block_start_=now_ms;
    }
    return true;
  }

  float max_std_rad_s() const {
    float maximum=0.0f;
    if (samples_<2U) return maximum;
    for (unsigned i=0;i<3;++i) {
      const float variance=m2_[i]/static_cast<float>(samples_-1U);
      if (!std::isfinite(variance)) return INFINITY;
      if (variance>maximum) maximum=variance;
    }
    return std::sqrt(maximum);
  }

private:
  uint32_t samples_=0, block_samples_=0, block_start_=0;
  float mean_[3]={}, m2_[3]={}, block_sum_[3]={};
  float mean_min_[3]={}, mean_max_[3]={};
  bool have_block_=false;
};

#endif
