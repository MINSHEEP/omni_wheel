#ifndef LSLIDAR_DRIVER__SCAN_TIMING_HPP_
#define LSLIDAR_DRIVER__SCAN_TIMING_HPP_

#include <cmath>

namespace lslidar_driver {

struct ScanTiming {
  double first_bin_time;
  double scan_time;
};

// M10P 在方位角 0° 处分圈，而 LaserScan 的第一个槽位是 -180°。
// 该槽位约在一圈中点采集；串口断流或时钟跳变时不猜测采样时刻。
inline bool m10pScanTiming(double previous_end, double current_end,
                           double motor_frequency_hz, ScanTiming* result) {
  if (result == nullptr || !std::isfinite(previous_end) || !std::isfinite(current_end) ||
      !std::isfinite(motor_frequency_hz) || previous_end <= 0.0 ||
      motor_frequency_hz <= 0.0) {
    return false;
  }
  const double period = current_end - previous_end;
  const double nominal_period = 1.0 / motor_frequency_hz;
  if (period < 0.5 * nominal_period || period > 1.5 * nominal_period) {
    return false;
  }
  result->scan_time = period;
  result->first_bin_time = previous_end + 0.5 * period;
  return true;
}

}  // namespace lslidar_driver

#endif  // LSLIDAR_DRIVER__SCAN_TIMING_HPP_
