/*
 * div.c
 *
 * Code generation for function 'div'
 *
 */

/* Include files */
#include "div.h"
#include "PositioningSystem_V5_1_data.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
void binary_expand_op_2(creal_T in1[4], const creal_T in2[4], const creal_T in3,
                        const creal_T in4_data[], const int in4_size[2])
{
  double ai;
  double ar;
  double b_s;
  double bi;
  double br;
  double brm;
  double im;
  double re;
  int stride_0_1;
  stride_0_1 = (in4_size[1] != 1);
  ar = in2[0].re - in3.re;
  ai = in2[0].im - in3.im;
  br = in4_data[0].re;
  bi = in4_data[0].im;
  if (bi == 0.0) {
    if (ai == 0.0) {
      re = ar / br;
      im = 0.0;
    } else if (ar == 0.0) {
      re = 0.0;
      im = ai / br;
    } else {
      re = ar / br;
      im = ai / br;
    }
  } else if (br == 0.0) {
    if (ar == 0.0) {
      re = ai / bi;
      im = 0.0;
    } else if (ai == 0.0) {
      re = 0.0;
      im = -(ar / bi);
    } else {
      re = ai / bi;
      im = -(ar / bi);
    }
  } else {
    brm = fabs(br);
    im = fabs(bi);
    if (brm > im) {
      b_s = bi / br;
      im = br + b_s * bi;
      re = (ar + b_s * ai) / im;
      im = (ai - b_s * ar) / im;
    } else if (im == brm) {
      if (br > 0.0) {
        b_s = 0.5;
      } else {
        b_s = -0.5;
      }
      if (bi > 0.0) {
        im = 0.5;
      } else {
        im = -0.5;
      }
      re = (ar * b_s + ai * im) / brm;
      im = (ai * b_s - ar * im) / brm;
    } else {
      b_s = br / bi;
      im = bi + b_s * br;
      re = (b_s * ar + ai) / im;
      im = (b_s * ai - ar) / im;
    }
  }
  in1[0].re = re;
  in1[0].im = im;
  ar = in2[1].re - in3.re;
  ai = in2[1].im - in3.im;
  br = in4_data[stride_0_1].re;
  bi = in4_data[stride_0_1].im;
  if (bi == 0.0) {
    if (ai == 0.0) {
      re = ar / br;
      im = 0.0;
    } else if (ar == 0.0) {
      re = 0.0;
      im = ai / br;
    } else {
      re = ar / br;
      im = ai / br;
    }
  } else if (br == 0.0) {
    if (ar == 0.0) {
      re = ai / bi;
      im = 0.0;
    } else if (ai == 0.0) {
      re = 0.0;
      im = -(ar / bi);
    } else {
      re = ai / bi;
      im = -(ar / bi);
    }
  } else {
    brm = fabs(br);
    im = fabs(bi);
    if (brm > im) {
      b_s = bi / br;
      im = br + b_s * bi;
      re = (ar + b_s * ai) / im;
      im = (ai - b_s * ar) / im;
    } else if (im == brm) {
      if (br > 0.0) {
        b_s = 0.5;
      } else {
        b_s = -0.5;
      }
      if (bi > 0.0) {
        im = 0.5;
      } else {
        im = -0.5;
      }
      re = (ar * b_s + ai * im) / brm;
      im = (ai * b_s - ar * im) / brm;
    } else {
      b_s = br / bi;
      im = bi + b_s * br;
      re = (b_s * ar + ai) / im;
      im = (b_s * ai - ar) / im;
    }
  }
  in1[1].re = re;
  in1[1].im = im;
  ar = in2[2].re - in3.re;
  ai = in2[2].im - in3.im;
  br = in4_data[stride_0_1 << 1].re;
  bi = in4_data[stride_0_1 << 1].im;
  if (bi == 0.0) {
    if (ai == 0.0) {
      re = ar / br;
      im = 0.0;
    } else if (ar == 0.0) {
      re = 0.0;
      im = ai / br;
    } else {
      re = ar / br;
      im = ai / br;
    }
  } else if (br == 0.0) {
    if (ar == 0.0) {
      re = ai / bi;
      im = 0.0;
    } else if (ai == 0.0) {
      re = 0.0;
      im = -(ar / bi);
    } else {
      re = ai / bi;
      im = -(ar / bi);
    }
  } else {
    brm = fabs(br);
    im = fabs(bi);
    if (brm > im) {
      b_s = bi / br;
      im = br + b_s * bi;
      re = (ar + b_s * ai) / im;
      im = (ai - b_s * ar) / im;
    } else if (im == brm) {
      if (br > 0.0) {
        b_s = 0.5;
      } else {
        b_s = -0.5;
      }
      if (bi > 0.0) {
        im = 0.5;
      } else {
        im = -0.5;
      }
      re = (ar * b_s + ai * im) / brm;
      im = (ai * b_s - ar * im) / brm;
    } else {
      b_s = br / bi;
      im = bi + b_s * br;
      re = (b_s * ar + ai) / im;
      im = (b_s * ai - ar) / im;
    }
  }
  in1[2].re = re;
  in1[2].im = im;
  ar = in2[3].re - in3.re;
  ai = in2[3].im - in3.im;
  br = in4_data[3 * stride_0_1].re;
  bi = in4_data[3 * stride_0_1].im;
  if (bi == 0.0) {
    if (ai == 0.0) {
      re = ar / br;
      im = 0.0;
    } else if (ar == 0.0) {
      re = 0.0;
      im = ai / br;
    } else {
      re = ar / br;
      im = ai / br;
    }
  } else if (br == 0.0) {
    if (ar == 0.0) {
      re = ai / bi;
      im = 0.0;
    } else if (ai == 0.0) {
      re = 0.0;
      im = -(ar / bi);
    } else {
      re = ai / bi;
      im = -(ar / bi);
    }
  } else {
    brm = fabs(br);
    im = fabs(bi);
    if (brm > im) {
      b_s = bi / br;
      im = br + b_s * bi;
      re = (ar + b_s * ai) / im;
      im = (ai - b_s * ar) / im;
    } else if (im == brm) {
      if (br > 0.0) {
        b_s = 0.5;
      } else {
        b_s = -0.5;
      }
      if (bi > 0.0) {
        im = 0.5;
      } else {
        im = -0.5;
      }
      re = (ar * b_s + ai * im) / brm;
      im = (ai * b_s - ar * im) / brm;
    } else {
      b_s = br / bi;
      im = bi + b_s * br;
      re = (b_s * ar + ai) / im;
      im = (b_s * ai - ar) / im;
    }
  }
  in1[3].re = re;
  in1[3].im = im;
}

/* End of code generation (div.c) */
