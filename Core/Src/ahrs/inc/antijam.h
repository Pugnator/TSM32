#pragma once
#include "math3d.h"
#include <math.h>

#define NXP_MOTION_CAL_EEADDR 60
#define NXP_MOTION_CAL_SIZE 68

class MagneticJammingDetector
{
private:
  float jammingThreshold = 5.0f;
  bool jammingActive = false;
  int jammingCounter = 0;

  uint32_t stateTime;

  float cal[16]; // 0-8=offsets, 9=field strength, 10-15=soft iron map

public:
  MagneticJammingDetector(){};  
  void sample(VectorFloat &m);
};