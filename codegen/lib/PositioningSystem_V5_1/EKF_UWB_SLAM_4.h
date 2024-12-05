/*
 * EKF_UWB_SLAM_4.h
 *
 * Code generation for function 'EKF_UWB_SLAM_4'
 *
 */

#ifndef EKF_UWB_SLAM_4_H
#define EKF_UWB_SLAM_4_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void EKF_UWB_SLAM_4(const creal_T PosUWB, double HeadUWB, double dPosSLAM[3],
                    double dHeadSLAM[3], const double PosSLAMc[3],
                    creal_T *Posn, double Headn[3]);

void EKF_UWB_SLAM_4_init(void);

void b_EKF_UWB_SLAM_4(const double dPosSLAM[3], const double dHeadSLAM[3],
                      creal_T *Posn, double Headn[3]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (EKF_UWB_SLAM_4.h) */
