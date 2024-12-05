/*
 * PositioningSystem_V4_2_terminate.c
 *
 * Code generation for function 'PositioningSystem_V4_2_terminate'
 *
 */

/* Include files */
#include "PositioningSystem_V4_2_terminate.h"
#include "PositioningSystem_V4_2_data.h"
#include "UWBPosition_V4_1.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void PositioningSystem_V4_2_terminate(void)
{
  UWBPosition_V4_1_free();
  isInitialized_PositioningSystem_V4_2 = false;
}

/* End of code generation (PositioningSystem_V4_2_terminate.c) */
