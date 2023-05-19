#pragma once
#include <memory>

class KalmanFilter
{
public:
  KalmanFilter();
  ~KalmanFilter(){};

  float updateEstimate(float mea);
  float getEstimate();

private:
  void kalmanInit(float mea_e, float est_e, float q);

  void setMeasurementError(float mea_e);
  void setEstimateError(float est_e);
  void setProcessNoise(float q);
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
extern std::unique_ptr<kFilter> adcfilter;
