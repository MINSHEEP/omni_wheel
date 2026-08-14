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
  return correct_vector(raw, sign, deadband, Vector3{1.0, 1.0, 1.0});
}

Vector3 correct_vector(
  const Vector3 &raw, const Vector3 &sign, const Vector3 &deadband, const Vector3 &scale)
{
  Vector3 output{};
  for (std::size_t i = 0; i < output.size(); ++i)
  {
    const double corrected = raw[i] * sign[i];
    output[i] = std::isfinite(corrected) && std::isfinite(scale[i]) &&
      scale[i] > 0.0 && std::abs(corrected) > deadband[i] ? corrected * scale[i] : 0.0;
  }
  return output;
}

}  // namespace kiwi_base
