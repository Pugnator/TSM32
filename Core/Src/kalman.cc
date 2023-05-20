#include <math.h>
#include "kalman.h"

KalmanFilter::KalmanFilter()
{
  
}

void KalmanFilter::configure(float measurementError, float estimateError, float noise)
{
  _err_measure = measurementError;
  _err_estimate = estimateError;
  _q = noise;
}

float KalmanFilter::estimate()
{
  return _current_estimate;
}

float KalmanFilter::update(float measurement)
{
  _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + _kalman_gain * (measurement - _last_estimate);
  _err_estimate = (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;

  return _current_estimate;
}

void KalmanFilter::setMeasurementError(float measurementError)
{
  _err_measure = measurementError;
}

void KalmanFilter::setEstimateError(float estimateError)
{
  _err_estimate = estimateError;
}

void KalmanFilter::setProcessNoise(float q)
{
  _q = q;
}

float KalmanFilter::getKalmanGain()
{
  return _kalman_gain;
}

float KalmanFilter::getEstimateError()
{
  return _err_estimate;
}
