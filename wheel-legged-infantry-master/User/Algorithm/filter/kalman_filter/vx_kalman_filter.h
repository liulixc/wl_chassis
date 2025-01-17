#ifndef _VX_KALMAN_FILTER_H
#define _VX_KALMAN_FILTER_H

#include "stdint.h"
#include "main.h"
#include "kalman_filter.h"

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);

void chassis_vx_kalman_run(void);

#endif


