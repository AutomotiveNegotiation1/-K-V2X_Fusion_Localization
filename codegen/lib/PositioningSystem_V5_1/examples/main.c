/*
 * main.c
 *
 * Code generation for function 'main'
 *
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include files */
#include "main.h"
#include "PositioningSystem_V5_1.h"
#include "PositioningSystem_V5_1_terminate.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void argInit_1xd65_real_T(double result_data[], int result_size[2]);

static double argInit_real_T(void);

/* Function Definitions */
static void argInit_1xd65_real_T(double result_data[], int result_size[2])
{
  int idx1;
  /* Set the size of the array.
Change this size to the value that the application requires. */
  result_size[0] = 1;
  result_size[1] = 2;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 2; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result_data[idx1] = argInit_real_T();
  }
}

static double argInit_real_T(void)
{
  return 0.0;
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  /* The initialize function is being called automatically from your entry-point
   * function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_PositioningSystem_V5_1();
  /* Terminate the application.
You do not need to do this more than one time. */
  PositioningSystem_V5_1_terminate();
  return 0;
}

void main_PositioningSystem_V5_1(void)
{
  double PositionVector_data[65];
  double PositionOut[10];
  int PositionVector_size[2];
  /* Initialize function 'PositioningSystem_V5_1' input arguments. */
  /* Initialize function input argument 'PositionVector'. */
  argInit_1xd65_real_T(PositionVector_data, PositionVector_size);
  /* Call the entry-point 'PositioningSystem_V5_1'. */
  PositioningSystem_V5_1(PositionVector_data, PositionVector_size, PositionOut);
}

/* End of code generation (main.c) */
