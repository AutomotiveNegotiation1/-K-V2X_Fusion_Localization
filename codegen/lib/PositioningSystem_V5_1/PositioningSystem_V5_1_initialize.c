/*
 * PositioningSystem_V5_1_initialize.c
 *
 * Code generation for function 'PositioningSystem_V5_1_initialize'
 *
 */

/* Include files */
#include "PositioningSystem_V5_1_initialize.h"
#include "EKF_UWB_SLAM_4.h"
#include "EKF_UWB_SLAM_IMU_1.h"
#include "PositioningSystem_V5_1.h"
#include "PositioningSystem_V5_1_data.h"
#include "UWBPosition_V4_1.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void PositioningSystem_V5_1_initialize(void)
{
  PositioningSystem_V5_1_init();
  EKF_UWB_SLAM_IMU_1_init();
  EKF_UWB_SLAM_4_init();
  UWBPosition_V4_1_init();
  isInitialized_PositioningSystem_V5_1 = true;
}

/* End of code generation (PositioningSystem_V5_1_initialize.c) */
