/**
 * @file measurement_processing.cpp
 * @brief 实现三轴测量的符号修正和静止死区。
 */
#include "kiwi_base/measurement_processing.hpp"

#include <cstddef>
#include <cmath>

namespace kiwi_base
{

Vector3 correct_vector(
  const Vector3 &raw, const Vector3 &sign, const Vector3 &deadband)
{
  Vector3 output{};
  for (std::size_t i = 0; i < output.size(); ++i)
  {
    const double corrected = raw[i] * sign[i];
    output[i] = std::isfinite(corrected) && std::abs(corrected) > deadband[i] ?
      corrected : 0.0;
  }
  return output;
}

}  // namespace kiwi_base
