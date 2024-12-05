/*
 * PositioningSystem_V5_1_terminate.c
 *
 * Code generation for function 'PositioningSystem_V5_1_terminate'
 *
 */

/* Include files */
#include "PositioningSystem_V5_1_terminate.h"
#include "PositioningSystem_V5_1_data.h"
#include "UWBPosition_V4_1.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void PositioningSystem_V5_1_terminate(void)
{
  UWBPosition_V4_1_free();
  isInitialized_PositioningSystem_V5_1 = false;
}

/* End of code generation (PositioningSystem_V5_1_terminate.c) */
