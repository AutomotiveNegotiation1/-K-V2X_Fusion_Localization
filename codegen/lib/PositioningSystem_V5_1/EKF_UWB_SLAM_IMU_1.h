/*
 * EKF_UWB_SLAM_IMU_1.h
 *
 * Code generation for function 'EKF_UWB_SLAM_IMU_1'
 *
 */

#ifndef EKF_UWB_SLAM_IMU_1_H
#define EKF_UWB_SLAM_IMU_1_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double EKF_UWB_SLAM_IMU_1(const double acc[3], const double GyroD[3],
                          double IMUtime, const creal_T PosUWB, double HeadUWB,
                          double UWBtime, creal_T *PosHF, double *GammHF,
                          double *BetaHF);

void EKF_UWB_SLAM_IMU_1_init(void);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (EKF_UWB_SLAM_IMU_1.h) */
