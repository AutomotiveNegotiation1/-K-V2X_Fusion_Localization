/*
 * PositioningSystem_V4_2_initialize.c
 *
 * Code generation for function 'PositioningSystem_V4_2_initialize'
 *
 */

/* Include files */
#include "PositioningSystem_V4_2_initialize.h"
#include "EKF_UWB_SLAM_4.h"
#include "PositioningSystem_V4_2.h"
#include "PositioningSystem_V4_2_data.h"
#include "UWBPosition_V4_1.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void PositioningSystem_V4_2_initialize(void)
{
  PositioningSystem_V4_2_init();
  EKF_UWB_SLAM_4_init();
  UWBPosition_V4_1_init();
  isInitialized_PositioningSystem_V4_2 = true;
}

/* End of code generation (PositioningSystem_V4_2_initialize.c) */
