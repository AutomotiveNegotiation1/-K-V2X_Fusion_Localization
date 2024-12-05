/*
 * find.c
 *
 * Code generation for function 'find'
 *
 */

/* Include files */
#include "find.h"
#include "PositioningSystem_V4_2_emxutil.h"
#include "PositioningSystem_V4_2_types.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void eml_find(const boolean_T x[20000], emxArray_int32_T *i)
{
  int idx;
  int ii;
  int *i_data;
  boolean_T exitg1;
  idx = 0;
  ii = i->size[0] * i->size[1];
  i->size[0] = 1;
  i->size[1] = 20000;
  emxEnsureCapacity_int32_T(i, ii);
  i_data = i->data;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 20000)) {
    if (x[ii]) {
      idx++;
      i_data[idx - 1] = ii + 1;
      if (idx >= 20000) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  ii = i->size[0] * i->size[1];
  if (idx < 1) {
    i->size[1] = 0;
  } else {
    i->size[1] = idx;
  }
  emxEnsureCapacity_int32_T(i, ii);
}

/* End of code generation (find.c) */
