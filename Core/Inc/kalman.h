#pragma once
#include <memory>

class KalmanFilter
{
public:
  KalmanFilter();
  ~KalmanFilter(){};

  void configure(float measurementError, float estimateError, float noiseLevel);
  float update(float measurement);
  float estimate();

private:
  void setMeasurementError(float measurementError);
  void setEstimateError(float estimateError);
  void setProcessNoise(float noiseLevel);
  float getKalmanGain();
  float getEstimateError();

  float _err_measure;
  float _err_estimate;
  float _q;
  float _current_estimate;
  float _last_estimate;
  float _kalman_gain;
};

typedef struct KalmanFilter kFilter;
