/*
 * exp.c
 *
 * Code generation for function 'exp'
 *
 */

/* Include files */
#include "exp.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
void b_exp(creal_T *x)
{
  if (x->re == 0.0) {
    double d;
    d = x->im;
    x->re = cos(d);
    x->im = sin(d);
  } else if (x->im == 0.0) {
    x->re = exp(x->re);
    x->im = 0.0;
  } else if (rtIsInf(x->im) && rtIsInf(x->re) && (x->re < 0.0)) {
    x->re = 0.0;
    x->im = 0.0;
  } else {
    double d;
    double r;
    r = exp(x->re / 2.0);
    d = x->im;
    x->re = r * (r * cos(d));
    x->im = r * (r * sin(d));
  }
}

/* End of code generation (exp.c) */
