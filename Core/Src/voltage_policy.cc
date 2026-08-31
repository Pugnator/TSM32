#include "voltage_policy.h"

static_assert(VOLTAGE_FILTER_WINDOW_SIZE > 0,
              "VOLTAGE_FILTER_WINDOW_SIZE must be non-zero");

namespace VoltagePolicy
{

Controller::Controller()
    : values_{}, index_(0), sum_(0), seeded_(false), highTiming_(false),
      lowTiming_(false), highSince_(0), lowSince_(0)
{
}

Result Controller::update(uint32_t rawAdc, uint32_t now)
{
  if (!seeded_)
  {
    for (uint32_t &value : values_)
    {
      value = rawAdc;
      sum_ += rawAdc;
    }
    seeded_ = true;
  }
  else
  {
    sum_ -= values_[index_];
    values_[index_] = rawAdc;
    sum_ += rawAdc;
    index_ = (index_ + 1u) % VOLTAGE_FILTER_WINDOW_SIZE;
  }

  const uint32_t filtered = static_cast<uint32_t>(
      sum_ / VOLTAGE_FILTER_WINDOW_SIZE);
  Region region = Region::Middle;
  if (filtered > ADC_13_4V_VALUE)
  {
    region = Region::High;
  }
  else if (filtered <= ADC_11_1V_VALUE)
  {
    region = Region::Low;
  }

  bool highQualified = false;
  bool lowQualified = false;
  switch (region)
  {
  case Region::High:
    lowTiming_ = false;
    if (!highTiming_)
    {
      highTiming_ = true;
      highSince_ = now;
    }
    highQualified = (now - highSince_) >= VOLTAGE_DETECTION_THRESHOLD;
    break;

  case Region::Low:
    highTiming_ = false;
    if (!lowTiming_)
    {
      lowTiming_ = true;
      lowSince_ = now;
    }
    lowQualified = (now - lowSince_) >= LOW_VOLTAGE_DETECTION_THRESHOLD;
    break;

  case Region::Middle:
    highTiming_ = false;
    lowTiming_ = false;
    break;
  }

  return {filtered, region, highQualified, lowQualified};
}

} // namespace VoltagePolicy
