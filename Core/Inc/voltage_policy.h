#pragma once

#include <stdint.h>

#include "settings.h"

namespace VoltagePolicy
{

enum class Region : uint8_t
{
  Low,
  Middle,
  High,
};

struct Result
{
  uint32_t filteredAdc;
  Region region;
  bool highQualified;
  bool lowQualified;
};

/* Short noise filter plus continuous high/low qualification. The two timers
 * are independent and reset whenever their region is left, including entry
 * into the hysteresis middle band. */
class Controller
{
public:
  Controller();
  Result update(uint32_t rawAdc, uint32_t now);

private:
  uint32_t values_[VOLTAGE_FILTER_WINDOW_SIZE];
  uint32_t index_;
  uint64_t sum_;
  bool seeded_;
  bool highTiming_;
  bool lowTiming_;
  uint32_t highSince_;
  uint32_t lowSince_;
};

} // namespace VoltagePolicy
