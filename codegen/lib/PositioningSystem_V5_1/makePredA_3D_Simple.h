/*
 * makePredA_3D_Simple.h
 *
 * Code generation for function 'makePredA_3D_Simple'
 *
 */

#ifndef MAKEPREDA_3D_SIMPLE_H
#define MAKEPREDA_3D_SIMPLE_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void makePredA_3D_Simple(const double Xhat[15], const double acc[3],
                         const double p[3], double A[225]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (makePredA_3D_Simple.h) */
