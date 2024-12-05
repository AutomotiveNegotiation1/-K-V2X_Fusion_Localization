/*
 * sortIdx.h
 *
 * Code generation for function 'sortIdx'
 *
 */

#ifndef SORTIDX_H
#define SORTIDX_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void merge_block(int idx[20000], double x[20000], int offset, int n,
                 int preSortLevel, int iwork[20000], double xwork[20000]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (sortIdx.h) */
