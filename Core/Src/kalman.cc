#include <math.h>
#include "kalman.h"

  KalmanFilter::KalmanFilter()
  {
    kalmanInit(2, 2, 0.01f);
  }

  void KalmanFilter::kalmanInit(float mea_e, float est_e, float q)
  {
    _err_measure = mea_e;
    _err_estimate = est_e;
    _q = q;
  }

  float KalmanFilter::getEstimate()
  {
    return _current_estimate;
  }

  float KalmanFilter::updateEstimate(float mea)
  {
    _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
    _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
    _err_estimate = (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
    _last_estimate = _current_estimate;

    return _current_estimate;
  }

  void KalmanFilter::setMeasurementError(float mea_e)
  {
    _err_measure = mea_e;
  }

  void KalmanFilter::setEstimateError(float est_e)
  {
    _err_estimate = est_e;
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
