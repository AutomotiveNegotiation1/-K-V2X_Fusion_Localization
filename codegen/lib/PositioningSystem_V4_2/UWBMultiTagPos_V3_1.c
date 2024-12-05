/*
 * UWBMultiTagPos_V3_1.c
 *
 * Code generation for function 'UWBMultiTagPos_V3_1'
 *
 */

/* Include files */
#include "UWBMultiTagPos_V3_1.h"
#include "PositioningSystem_V4_2_data.h"
#include "PositioningSystem_V4_2_emxutil.h"
#include "PositioningSystem_V4_2_rtwutil.h"
#include "PositioningSystem_V4_2_types.h"
#include "UWBPosition_V4_1.h"
#include "abs.h"
#include "div.h"
#include "find.h"
#include "mean.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static void binary_expand_op_2(emxArray_real_T *in1,
                               const emxArray_real_T *in2);

static void binary_expand_op_5(emxArray_real_T *in1, const emxArray_real_T *in2,
                               const emxArray_real_T *in3);

static void binary_expand_op_6(emxArray_creal_T *in1, const double in3_data[],
                               const int in3_size[2], const double in5_data[],
                               const int in5_size[2]);

/* Function Definitions */
static void binary_expand_op_2(emxArray_real_T *in1, const emxArray_real_T *in2)
{
  emxArray_real_T *b_in2;
  const double *in2_data;
  double *b_in2_data;
  double *in1_data;
  int aux_0_1;
  int aux_1_1;
  int i;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in2_data = in2->data;
  in1_data = in1->data;
  emxInit_real_T(&b_in2, 2);
  i = b_in2->size[0] * b_in2->size[1];
  b_in2->size[0] = 4;
  if (in1->size[1] == 1) {
    loop_ub = in2->size[1];
  } else {
    loop_ub = in1->size[1];
  }
  b_in2->size[1] = loop_ub;
  emxEnsureCapacity_real_T(b_in2, i);
  b_in2_data = b_in2->data;
  stride_0_1 = (in2->size[1] != 1);
  stride_1_1 = (in1->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    b_in2_data[4 * i] =
        in2_data[4 * aux_0_1 + 4 * in2->size[1] * 99] - in1_data[4 * aux_1_1];
    b_in2_data[4 * i + 1] =
        in2_data[(4 * aux_0_1 + 4 * in2->size[1] * 99) + 1] -
        in1_data[4 * aux_1_1 + 1];
    b_in2_data[4 * i + 2] =
        in2_data[(4 * aux_0_1 + 4 * in2->size[1] * 99) + 2] -
        in1_data[4 * aux_1_1 + 2];
    b_in2_data[4 * i + 3] =
        in2_data[(4 * aux_0_1 + 4 * in2->size[1] * 99) + 3] -
        in1_data[4 * aux_1_1 + 3];
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  i = in1->size[0] * in1->size[1];
  in1->size[0] = 4;
  in1->size[1] = b_in2->size[1];
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  loop_ub = b_in2->size[1];
  for (i = 0; i < loop_ub; i++) {
    in1_data[4 * i] = b_in2_data[4 * i];
    stride_0_1 = 4 * i + 1;
    in1_data[stride_0_1] = b_in2_data[stride_0_1];
    stride_0_1 = 4 * i + 2;
    in1_data[stride_0_1] = b_in2_data[stride_0_1];
    stride_0_1 = 4 * i + 3;
    in1_data[stride_0_1] = b_in2_data[stride_0_1];
  }
  emxFree_real_T(&b_in2);
}

static void binary_expand_op_5(emxArray_real_T *in1, const emxArray_real_T *in2,
                               const emxArray_real_T *in3)
{
  const double *in2_data;
  const double *in3_data;
  double *in1_data;
  int aux_0_1;
  int aux_1_1;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_0_1;
  int stride_1_1;
  in3_data = in3->data;
  in2_data = in2->data;
  i = in1->size[0] * in1->size[1];
  in1->size[0] = 4;
  emxEnsureCapacity_real_T(in1, i);
  if (in3->size[1] == 1) {
    loop_ub = in2->size[1];
  } else {
    loop_ub = in3->size[1];
  }
  i = in1->size[0] * in1->size[1];
  in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_0_1 = (in2->size[1] != 1);
  stride_1_1 = (in3->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    in1_data[4 * i] = in2_data[in2->size[0] * aux_0_1] -
                      in3_data[4 * aux_1_1 + 4 * in3->size[1] * 99];
    in1_data[4 * i + 1] = in2_data[stride_0_0 + in2->size[0] * aux_0_1] -
                          in3_data[(4 * aux_1_1 + 4 * in3->size[1] * 99) + 1];
    in1_data[4 * i + 2] = in2_data[(stride_0_0 << 1) + in2->size[0] * aux_0_1] -
                          in3_data[(4 * aux_1_1 + 4 * in3->size[1] * 99) + 2];
    in1_data[4 * i + 3] = in2_data[3 * stride_0_0 + in2->size[0] * aux_0_1] -
                          in3_data[(4 * aux_1_1 + 4 * in3->size[1] * 99) + 3];
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
}

static void binary_expand_op_6(emxArray_creal_T *in1, const double in3_data[],
                               const int in3_size[2], const double in5_data[],
                               const int in5_size[2])
{
  creal_T *in1_data;
  int i;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1_data = in1->data;
  stride_0_1 = (in3_size[1] != 1);
  stride_1_1 = (in5_size[1] != 1);
  if (in5_size[1] == 1) {
    loop_ub = in3_size[1];
  } else {
    loop_ub = in5_size[1];
  }
  for (i = 0; i < loop_ub; i++) {
    double d;
    d = in5_data[i * stride_1_1];
    in1_data[i].re = in3_data[i * stride_0_1] + 0.0 * d;
    in1_data[i].im = d;
  }
}

double UWBMultiTagPos_V3_1(const cell_wrap_4 b_PosUWB2[4],
                           const double xt_b_data[], const int xt_b_size[2],
                           const double yt_b_data[], const int yt_b_size[2],
                           const double xain_data[], const int xain_size[2],
                           const double yain_data[], const int yain_size[2],
                           const emxArray_real_T *b_DistMap, double Ln,
                           creal_T *PosHH)
{
  static creal_T TagCandA[80000];
  static creal_T CenterCands[20000];
  static creal_T HeadingCands[20000];
  static double y[20000];
  static unsigned int LessThan1m1[20000];
  emxArray_boolean_T *b_x;
  emxArray_boolean_T *r2;
  emxArray_creal_T *Xain;
  emxArray_creal_T *Xt_b;
  emxArray_creal_T *sortedCenter;
  emxArray_creal_T *sortedHeading;
  emxArray_creal_T *x;
  emxArray_int32_T *b_i;
  emxArray_int32_T *r;
  emxArray_real_T *DistErr;
  emxArray_real_T *TempA;
  emxArray_real_T *TempB;
  emxArray_real_T *b_y;
  emxArray_real_T *r1;
  creal_T Xt_b_data[65];
  creal_T Temp[4];
  creal_T z[4];
  creal_T CenterCandA;
  creal_T HeadingCandA;
  creal_T *Xain_data;
  creal_T *sortedCenter_data;
  const double *DistMap_data;
  double HeadingCandA_tmp;
  double HeadingHH;
  double PrevAbsHeadingCandA;
  double PrevHeadingCandA_im;
  double PrevHeadingCandA_re;
  double brm;
  double bsum;
  double *DistErr_data;
  double *TempA_data;
  double *TempB_data;
  int Xt_b_size[2];
  int i;
  int i1;
  int i2;
  int idx;
  int k;
  int k0;
  int kk;
  int lastBlockLength;
  int ll;
  int mm;
  int nn;
  int nx;
  int ss;
  int *i_data;
  boolean_T b_Temp_tmp;
  boolean_T *x_data;
  DistMap_data = b_DistMap->data;
  if (xt_b_size[1] == yt_b_size[1]) {
    Xt_b_size[0] = 1;
    Xt_b_size[1] = xt_b_size[1];
    lastBlockLength = xt_b_size[1];
    for (i = 0; i < lastBlockLength; i++) {
      HeadingCandA_tmp = yt_b_data[i];
      Xt_b_data[i].re = xt_b_data[i] + 0.0 * HeadingCandA_tmp;
      Xt_b_data[i].im = HeadingCandA_tmp;
    }
  } else {
    binary_expand_op_1(Xt_b_data, Xt_b_size, xt_b_data, xt_b_size, yt_b_data,
                       yt_b_size);
  }
  emxInit_creal_T(&Xain);
  i = Xain->size[0] * Xain->size[1];
  Xain->size[0] = 1;
  k0 = (int)Ln;
  Xain->size[1] = (int)Ln;
  emxEnsureCapacity_creal_T(Xain, i);
  Xain_data = Xain->data;
  for (i = 0; i < k0; i++) {
    Xain_data[i].re = 0.0;
    Xain_data[i].im = 0.0;
  }
  if (xain_size[1] == yain_size[1]) {
    lastBlockLength = xain_size[1];
    for (i = 0; i < lastBlockLength; i++) {
      HeadingCandA_tmp = yain_data[i];
      Xain_data[i].re = xain_data[i] + 0.0 * HeadingCandA_tmp;
      Xain_data[i].im = HeadingCandA_tmp;
    }
  } else {
    binary_expand_op_6(Xain, xain_data, xain_size, yain_data, yain_size);
    Xain_data = Xain->data;
  }
  memset(&HeadingCands[0], 0, 20000U * sizeof(creal_T));
  memset(&CenterCands[0], 0, 20000U * sizeof(creal_T));
  memset(&TagCandA[0], 0, 80000U * sizeof(creal_T));
  memset(&LessThan1m1[0], 0, 20000U * sizeof(unsigned int));
  ss = 0;
  PrevAbsHeadingCandA = 100.0;
  PosHH->re = 0.0;
  PosHH->im = 0.0;
  PrevHeadingCandA_re = 0.0;
  PrevHeadingCandA_im = 0.0;
  i = b_PosUWB2[0].f1.size[1];
  emxInit_real_T(&DistErr, 2);
  emxInit_real_T(&TempA, 2);
  emxInit_creal_T(&x);
  emxInit_int32_T(&b_i, 1);
  emxInit_boolean_T(&b_x, 1);
  for (kk = 0; kk < i; kk++) {
    Temp[0] = b_PosUWB2[0].f1.data[kk];
    i1 = b_PosUWB2[1].f1.size[1];
    for (ll = 0; ll < i1; ll++) {
      Temp[1] = b_PosUWB2[1].f1.data[ll];
      i2 = b_PosUWB2[2].f1.size[1];
      for (mm = 0; mm < i2; mm++) {
        int i3;
        Temp[2] = b_PosUWB2[2].f1.data[mm];
        i3 = b_PosUWB2[3].f1.size[1];
        for (nn = 0; nn < i3; nn++) {
          double PosUWB2_tmp;
          Temp[3] = b_PosUWB2[3].f1.data[nn];
          k0 = 0;
          if ((Temp[0].re == 0.0) && (Temp[0].im == 0.0)) {
            k0 = 1;
          }
          if ((Temp[1].re == 0.0) && (Temp[1].im == 0.0)) {
            k0++;
          }
          if ((Temp[2].re == 0.0) && (Temp[2].im == 0.0)) {
            k0++;
          }
          if ((Temp[3].re == 0.0) && (Temp[3].im == 0.0)) {
            k0++;
          }
          if (k0 == 0) {
            HeadingCandA_tmp =
                ((Temp[0].re + Temp[1].re) + Temp[2].re) + Temp[3].re;
            bsum = ((Temp[0].im + Temp[1].im) + Temp[2].im) + Temp[3].im;
            if (bsum == 0.0) {
              CenterCandA.re = HeadingCandA_tmp / 4.0;
              CenterCandA.im = 0.0;
            } else if (HeadingCandA_tmp == 0.0) {
              CenterCandA.re = 0.0;
              CenterCandA.im = bsum / 4.0;
            } else {
              CenterCandA.re = HeadingCandA_tmp / 4.0;
              CenterCandA.im = bsum / 4.0;
            }
            if (Xt_b_size[1] == 4) {
              double ai;
              double ar;
              ar = Temp[0].re - CenterCandA.re;
              ai = Temp[0].im - CenterCandA.im;
              if (Xt_b_data[0].im == 0.0) {
                if (ai == 0.0) {
                  z[0].re = ar / Xt_b_data[0].re;
                  z[0].im = 0.0;
                } else if (ar == 0.0) {
                  z[0].re = 0.0;
                  z[0].im = ai / Xt_b_data[0].re;
                } else {
                  z[0].re = ar / Xt_b_data[0].re;
                  z[0].im = ai / Xt_b_data[0].re;
                }
              } else if (Xt_b_data[0].re == 0.0) {
                if (ar == 0.0) {
                  z[0].re = ai / Xt_b_data[0].im;
                  z[0].im = 0.0;
                } else if (ai == 0.0) {
                  z[0].re = 0.0;
                  z[0].im = -(ar / Xt_b_data[0].im);
                } else {
                  z[0].re = ai / Xt_b_data[0].im;
                  z[0].im = -(ar / Xt_b_data[0].im);
                }
              } else {
                brm = fabs(Xt_b_data[0].re);
                bsum = fabs(Xt_b_data[0].im);
                if (brm > bsum) {
                  HeadingCandA_tmp = Xt_b_data[0].im / Xt_b_data[0].re;
                  bsum = Xt_b_data[0].re + HeadingCandA_tmp * Xt_b_data[0].im;
                  z[0].re = (ar + HeadingCandA_tmp * ai) / bsum;
                  z[0].im = (ai - HeadingCandA_tmp * ar) / bsum;
                } else if (bsum == brm) {
                  if (Xt_b_data[0].re > 0.0) {
                    HeadingCandA_tmp = 0.5;
                  } else {
                    HeadingCandA_tmp = -0.5;
                  }
                  if (Xt_b_data[0].im > 0.0) {
                    bsum = 0.5;
                  } else {
                    bsum = -0.5;
                  }
                  z[0].re = (ar * HeadingCandA_tmp + ai * bsum) / brm;
                  z[0].im = (ai * HeadingCandA_tmp - ar * bsum) / brm;
                } else {
                  HeadingCandA_tmp = Xt_b_data[0].re / Xt_b_data[0].im;
                  bsum = Xt_b_data[0].im + HeadingCandA_tmp * Xt_b_data[0].re;
                  z[0].re = (HeadingCandA_tmp * ar + ai) / bsum;
                  z[0].im = (HeadingCandA_tmp * ai - ar) / bsum;
                }
              }
              ar = Temp[1].re - CenterCandA.re;
              ai = Temp[1].im - CenterCandA.im;
              if (Xt_b_data[1].im == 0.0) {
                if (ai == 0.0) {
                  z[1].re = ar / Xt_b_data[1].re;
                  z[1].im = 0.0;
                } else if (ar == 0.0) {
                  z[1].re = 0.0;
                  z[1].im = ai / Xt_b_data[1].re;
                } else {
                  z[1].re = ar / Xt_b_data[1].re;
                  z[1].im = ai / Xt_b_data[1].re;
                }
              } else if (Xt_b_data[1].re == 0.0) {
                if (ar == 0.0) {
                  z[1].re = ai / Xt_b_data[1].im;
                  z[1].im = 0.0;
                } else if (ai == 0.0) {
                  z[1].re = 0.0;
                  z[1].im = -(ar / Xt_b_data[1].im);
                } else {
                  z[1].re = ai / Xt_b_data[1].im;
                  z[1].im = -(ar / Xt_b_data[1].im);
                }
              } else {
                brm = fabs(Xt_b_data[1].re);
                bsum = fabs(Xt_b_data[1].im);
                if (brm > bsum) {
                  HeadingCandA_tmp = Xt_b_data[1].im / Xt_b_data[1].re;
                  bsum = Xt_b_data[1].re + HeadingCandA_tmp * Xt_b_data[1].im;
                  z[1].re = (ar + HeadingCandA_tmp * ai) / bsum;
                  z[1].im = (ai - HeadingCandA_tmp * ar) / bsum;
                } else if (bsum == brm) {
                  if (Xt_b_data[1].re > 0.0) {
                    HeadingCandA_tmp = 0.5;
                  } else {
                    HeadingCandA_tmp = -0.5;
                  }
                  if (Xt_b_data[1].im > 0.0) {
                    bsum = 0.5;
                  } else {
                    bsum = -0.5;
                  }
                  z[1].re = (ar * HeadingCandA_tmp + ai * bsum) / brm;
                  z[1].im = (ai * HeadingCandA_tmp - ar * bsum) / brm;
                } else {
                  HeadingCandA_tmp = Xt_b_data[1].re / Xt_b_data[1].im;
                  bsum = Xt_b_data[1].im + HeadingCandA_tmp * Xt_b_data[1].re;
                  z[1].re = (HeadingCandA_tmp * ar + ai) / bsum;
                  z[1].im = (HeadingCandA_tmp * ai - ar) / bsum;
                }
              }
              ar = Temp[2].re - CenterCandA.re;
              ai = Temp[2].im - CenterCandA.im;
              if (Xt_b_data[2].im == 0.0) {
                if (ai == 0.0) {
                  z[2].re = ar / Xt_b_data[2].re;
                  z[2].im = 0.0;
                } else if (ar == 0.0) {
                  z[2].re = 0.0;
                  z[2].im = ai / Xt_b_data[2].re;
                } else {
                  z[2].re = ar / Xt_b_data[2].re;
                  z[2].im = ai / Xt_b_data[2].re;
                }
              } else if (Xt_b_data[2].re == 0.0) {
                if (ar == 0.0) {
                  z[2].re = ai / Xt_b_data[2].im;
                  z[2].im = 0.0;
                } else if (ai == 0.0) {
                  z[2].re = 0.0;
                  z[2].im = -(ar / Xt_b_data[2].im);
                } else {
                  z[2].re = ai / Xt_b_data[2].im;
                  z[2].im = -(ar / Xt_b_data[2].im);
                }
              } else {
                brm = fabs(Xt_b_data[2].re);
                bsum = fabs(Xt_b_data[2].im);
                if (brm > bsum) {
                  HeadingCandA_tmp = Xt_b_data[2].im / Xt_b_data[2].re;
                  bsum = Xt_b_data[2].re + HeadingCandA_tmp * Xt_b_data[2].im;
                  z[2].re = (ar + HeadingCandA_tmp * ai) / bsum;
                  z[2].im = (ai - HeadingCandA_tmp * ar) / bsum;
                } else if (bsum == brm) {
                  if (Xt_b_data[2].re > 0.0) {
                    HeadingCandA_tmp = 0.5;
                  } else {
                    HeadingCandA_tmp = -0.5;
                  }
                  if (Xt_b_data[2].im > 0.0) {
                    bsum = 0.5;
                  } else {
                    bsum = -0.5;
                  }
                  z[2].re = (ar * HeadingCandA_tmp + ai * bsum) / brm;
                  z[2].im = (ai * HeadingCandA_tmp - ar * bsum) / brm;
                } else {
                  HeadingCandA_tmp = Xt_b_data[2].re / Xt_b_data[2].im;
                  bsum = Xt_b_data[2].im + HeadingCandA_tmp * Xt_b_data[2].re;
                  z[2].re = (HeadingCandA_tmp * ar + ai) / bsum;
                  z[2].im = (HeadingCandA_tmp * ai - ar) / bsum;
                }
              }
              ar = Temp[3].re - CenterCandA.re;
              ai = Temp[3].im - CenterCandA.im;
              if (Xt_b_data[3].im == 0.0) {
                if (ai == 0.0) {
                  z[3].re = ar / Xt_b_data[3].re;
                  z[3].im = 0.0;
                } else if (ar == 0.0) {
                  z[3].re = 0.0;
                  z[3].im = ai / Xt_b_data[3].re;
                } else {
                  z[3].re = ar / Xt_b_data[3].re;
                  z[3].im = ai / Xt_b_data[3].re;
                }
              } else if (Xt_b_data[3].re == 0.0) {
                if (ar == 0.0) {
                  z[3].re = ai / Xt_b_data[3].im;
                  z[3].im = 0.0;
                } else if (ai == 0.0) {
                  z[3].re = 0.0;
                  z[3].im = -(ar / Xt_b_data[3].im);
                } else {
                  z[3].re = ai / Xt_b_data[3].im;
                  z[3].im = -(ar / Xt_b_data[3].im);
                }
              } else {
                brm = fabs(Xt_b_data[3].re);
                bsum = fabs(Xt_b_data[3].im);
                if (brm > bsum) {
                  HeadingCandA_tmp = Xt_b_data[3].im / Xt_b_data[3].re;
                  bsum = Xt_b_data[3].re + HeadingCandA_tmp * Xt_b_data[3].im;
                  z[3].re = (ar + HeadingCandA_tmp * ai) / bsum;
                  z[3].im = (ai - HeadingCandA_tmp * ar) / bsum;
                } else if (bsum == brm) {
                  if (Xt_b_data[3].re > 0.0) {
                    HeadingCandA_tmp = 0.5;
                  } else {
                    HeadingCandA_tmp = -0.5;
                  }
                  if (Xt_b_data[3].im > 0.0) {
                    bsum = 0.5;
                  } else {
                    bsum = -0.5;
                  }
                  z[3].re = (ar * HeadingCandA_tmp + ai * bsum) / brm;
                  z[3].im = (ai * HeadingCandA_tmp - ar * bsum) / brm;
                } else {
                  HeadingCandA_tmp = Xt_b_data[3].re / Xt_b_data[3].im;
                  bsum = Xt_b_data[3].im + HeadingCandA_tmp * Xt_b_data[3].re;
                  z[3].re = (HeadingCandA_tmp * ar + ai) / bsum;
                  z[3].im = (HeadingCandA_tmp * ai - ar) / bsum;
                }
              }
            } else {
              binary_expand_op_3(z, Temp, CenterCandA, Xt_b_data, Xt_b_size);
            }
            HeadingCandA_tmp = ((z[0].re + z[1].re) + z[2].re) + z[3].re;
            bsum = ((z[0].im + z[1].im) + z[2].im) + z[3].im;
            if (bsum == 0.0) {
              HeadingCandA_tmp /= 4.0;
              bsum = 0.0;
            } else if (HeadingCandA_tmp == 0.0) {
              HeadingCandA_tmp = 0.0;
              bsum /= 4.0;
            } else {
              HeadingCandA_tmp /= 4.0;
              bsum /= 4.0;
            }
            HeadingCandA.re = HeadingCandA_tmp;
            HeadingCandA.im = bsum;
          } else {
            creal_T b_z[2];
            double ai;
            double ar;
            double b_PosUWB2_tmp;
            double bi;
            double br;
            boolean_T Temp_tmp;
            boolean_T guard1;
            boolean_T guard2;
            boolean_T guard3;
            Temp_tmp = ((Temp[0].re != 0.0) || (Temp[0].im != 0.0));
            guard1 = false;
            guard2 = false;
            guard3 = false;
            if (Temp_tmp) {
              PosUWB2_tmp = b_PosUWB2[3].f1.data[nn].re;
              b_PosUWB2_tmp = b_PosUWB2[3].f1.data[nn].im;
              if ((PosUWB2_tmp != 0.0) || (b_PosUWB2_tmp != 0.0)) {
                ar = Temp[0].re + PosUWB2_tmp;
                ai = Temp[0].im + b_PosUWB2_tmp;
                if (ai == 0.0) {
                  CenterCandA.re = ar / 2.0;
                  CenterCandA.im = 0.0;
                } else if (ar == 0.0) {
                  CenterCandA.re = 0.0;
                  CenterCandA.im = ai / 2.0;
                } else {
                  CenterCandA.re = ar / 2.0;
                  CenterCandA.im = ai / 2.0;
                }
                ar = Temp[0].re - CenterCandA.re;
                ai = Temp[0].im - CenterCandA.im;
                if (Xt_b_data[0].im == 0.0) {
                  if (ai == 0.0) {
                    b_z[0].re = ar / Xt_b_data[0].re;
                    b_z[0].im = 0.0;
                  } else if (ar == 0.0) {
                    b_z[0].re = 0.0;
                    b_z[0].im = ai / Xt_b_data[0].re;
                  } else {
                    b_z[0].re = ar / Xt_b_data[0].re;
                    b_z[0].im = ai / Xt_b_data[0].re;
                  }
                } else if (Xt_b_data[0].re == 0.0) {
                  if (ar == 0.0) {
                    b_z[0].re = ai / Xt_b_data[0].im;
                    b_z[0].im = 0.0;
                  } else if (ai == 0.0) {
                    b_z[0].re = 0.0;
                    b_z[0].im = -(ar / Xt_b_data[0].im);
                  } else {
                    b_z[0].re = ai / Xt_b_data[0].im;
                    b_z[0].im = -(ar / Xt_b_data[0].im);
                  }
                } else {
                  brm = fabs(Xt_b_data[0].re);
                  bsum = fabs(Xt_b_data[0].im);
                  if (brm > bsum) {
                    HeadingCandA_tmp = Xt_b_data[0].im / Xt_b_data[0].re;
                    bsum = Xt_b_data[0].re + HeadingCandA_tmp * Xt_b_data[0].im;
                    b_z[0].re = (ar + HeadingCandA_tmp * ai) / bsum;
                    b_z[0].im = (ai - HeadingCandA_tmp * ar) / bsum;
                  } else if (bsum == brm) {
                    if (Xt_b_data[0].re > 0.0) {
                      HeadingCandA_tmp = 0.5;
                    } else {
                      HeadingCandA_tmp = -0.5;
                    }
                    if (Xt_b_data[0].im > 0.0) {
                      bsum = 0.5;
                    } else {
                      bsum = -0.5;
                    }
                    b_z[0].re = (ar * HeadingCandA_tmp + ai * bsum) / brm;
                    b_z[0].im = (ai * HeadingCandA_tmp - ar * bsum) / brm;
                  } else {
                    HeadingCandA_tmp = Xt_b_data[0].re / Xt_b_data[0].im;
                    bsum = Xt_b_data[0].im + HeadingCandA_tmp * Xt_b_data[0].re;
                    b_z[0].re = (HeadingCandA_tmp * ar + ai) / bsum;
                    b_z[0].im = (HeadingCandA_tmp * ai - ar) / bsum;
                  }
                }
                ar = Temp[3].re - CenterCandA.re;
                ai = Temp[3].im - CenterCandA.im;
                if (Xt_b_data[3].im == 0.0) {
                  if (ai == 0.0) {
                    b_z[1].re = ar / Xt_b_data[3].re;
                    b_z[1].im = 0.0;
                  } else if (ar == 0.0) {
                    b_z[1].re = 0.0;
                    b_z[1].im = ai / Xt_b_data[3].re;
                  } else {
                    b_z[1].re = ar / Xt_b_data[3].re;
                    b_z[1].im = ai / Xt_b_data[3].re;
                  }
                } else if (Xt_b_data[3].re == 0.0) {
                  if (ar == 0.0) {
                    b_z[1].re = ai / Xt_b_data[3].im;
                    b_z[1].im = 0.0;
                  } else if (ai == 0.0) {
                    b_z[1].re = 0.0;
                    b_z[1].im = -(ar / Xt_b_data[3].im);
                  } else {
                    b_z[1].re = ai / Xt_b_data[3].im;
                    b_z[1].im = -(ar / Xt_b_data[3].im);
                  }
                } else {
                  brm = fabs(Xt_b_data[3].re);
                  bsum = fabs(Xt_b_data[3].im);
                  if (brm > bsum) {
                    HeadingCandA_tmp = Xt_b_data[3].im / Xt_b_data[3].re;
                    bsum = Xt_b_data[3].re + HeadingCandA_tmp * Xt_b_data[3].im;
                    b_z[1].re = (ar + HeadingCandA_tmp * ai) / bsum;
                    b_z[1].im = (ai - HeadingCandA_tmp * ar) / bsum;
                  } else if (bsum == brm) {
                    if (Xt_b_data[3].re > 0.0) {
                      HeadingCandA_tmp = 0.5;
                    } else {
                      HeadingCandA_tmp = -0.5;
                    }
                    if (Xt_b_data[3].im > 0.0) {
                      bsum = 0.5;
                    } else {
                      bsum = -0.5;
                    }
                    b_z[1].re = (ar * HeadingCandA_tmp + ai * bsum) / brm;
                    b_z[1].im = (ai * HeadingCandA_tmp - ar * bsum) / brm;
                  } else {
                    HeadingCandA_tmp = Xt_b_data[3].re / Xt_b_data[3].im;
                    bsum = Xt_b_data[3].im + HeadingCandA_tmp * Xt_b_data[3].re;
                    b_z[1].re = (HeadingCandA_tmp * ar + ai) / bsum;
                    b_z[1].im = (HeadingCandA_tmp * ai - ar) / bsum;
                  }
                }
                ar = b_z[0].re + b_z[1].re;
                ai = b_z[0].im + b_z[1].im;
                if (ai == 0.0) {
                  HeadingCandA.re = ar / 2.0;
                  HeadingCandA.im = 0.0;
                } else if (ar == 0.0) {
                  HeadingCandA.re = 0.0;
                  HeadingCandA.im = ai / 2.0;
                } else {
                  HeadingCandA.re = ar / 2.0;
                  HeadingCandA.im = ai / 2.0;
                }
              } else {
                guard3 = true;
              }
            } else {
              guard3 = true;
            }
            if (guard3) {
              b_Temp_tmp = ((Temp[1].re != 0.0) || (Temp[1].im != 0.0));
              if (b_Temp_tmp && ((Temp[2].re != 0.0) || (Temp[2].im != 0.0))) {
                ar = Temp[1].re + Temp[2].re;
                ai = Temp[1].im + Temp[2].im;
                if (ai == 0.0) {
                  CenterCandA.re = ar / 2.0;
                  CenterCandA.im = 0.0;
                } else if (ar == 0.0) {
                  CenterCandA.re = 0.0;
                  CenterCandA.im = ai / 2.0;
                } else {
                  CenterCandA.re = ar / 2.0;
                  CenterCandA.im = ai / 2.0;
                }
                ar = Temp[1].re - CenterCandA.re;
                ai = Temp[1].im - CenterCandA.im;
                if (Xt_b_data[1].im == 0.0) {
                  if (ai == 0.0) {
                    b_z[0].re = ar / Xt_b_data[1].re;
                    b_z[0].im = 0.0;
                  } else if (ar == 0.0) {
                    b_z[0].re = 0.0;
                    b_z[0].im = ai / Xt_b_data[1].re;
                  } else {
                    b_z[0].re = ar / Xt_b_data[1].re;
                    b_z[0].im = ai / Xt_b_data[1].re;
                  }
                } else if (Xt_b_data[1].re == 0.0) {
                  if (ar == 0.0) {
                    b_z[0].re = ai / Xt_b_data[1].im;
                    b_z[0].im = 0.0;
                  } else if (ai == 0.0) {
                    b_z[0].re = 0.0;
                    b_z[0].im = -(ar / Xt_b_data[1].im);
                  } else {
                    b_z[0].re = ai / Xt_b_data[1].im;
                    b_z[0].im = -(ar / Xt_b_data[1].im);
                  }
                } else {
                  brm = fabs(Xt_b_data[1].re);
                  bsum = fabs(Xt_b_data[1].im);
                  if (brm > bsum) {
                    HeadingCandA_tmp = Xt_b_data[1].im / Xt_b_data[1].re;
                    bsum = Xt_b_data[1].re + HeadingCandA_tmp * Xt_b_data[1].im;
                    b_z[0].re = (ar + HeadingCandA_tmp * ai) / bsum;
                    b_z[0].im = (ai - HeadingCandA_tmp * ar) / bsum;
                  } else if (bsum == brm) {
                    if (Xt_b_data[1].re > 0.0) {
                      HeadingCandA_tmp = 0.5;
                    } else {
                      HeadingCandA_tmp = -0.5;
                    }
                    if (Xt_b_data[1].im > 0.0) {
                      bsum = 0.5;
                    } else {
                      bsum = -0.5;
                    }
                    b_z[0].re = (ar * HeadingCandA_tmp + ai * bsum) / brm;
                    b_z[0].im = (ai * HeadingCandA_tmp - ar * bsum) / brm;
                  } else {
                    HeadingCandA_tmp = Xt_b_data[1].re / Xt_b_data[1].im;
                    bsum = Xt_b_data[1].im + HeadingCandA_tmp * Xt_b_data[1].re;
                    b_z[0].re = (HeadingCandA_tmp * ar + ai) / bsum;
                    b_z[0].im = (HeadingCandA_tmp * ai - ar) / bsum;
                  }
                }
                ar = Temp[2].re - CenterCandA.re;
                ai = Temp[2].im - CenterCandA.im;
                if (Xt_b_data[2].im == 0.0) {
                  if (ai == 0.0) {
                    b_z[1].re = ar / Xt_b_data[2].re;
                    b_z[1].im = 0.0;
                  } else if (ar == 0.0) {
                    b_z[1].re = 0.0;
                    b_z[1].im = ai / Xt_b_data[2].re;
                  } else {
                    b_z[1].re = ar / Xt_b_data[2].re;
                    b_z[1].im = ai / Xt_b_data[2].re;
                  }
                } else if (Xt_b_data[2].re == 0.0) {
                  if (ar == 0.0) {
                    b_z[1].re = ai / Xt_b_data[2].im;
                    b_z[1].im = 0.0;
                  } else if (ai == 0.0) {
                    b_z[1].re = 0.0;
                    b_z[1].im = -(ar / Xt_b_data[2].im);
                  } else {
                    b_z[1].re = ai / Xt_b_data[2].im;
                    b_z[1].im = -(ar / Xt_b_data[2].im);
                  }
                } else {
                  brm = fabs(Xt_b_data[2].re);
                  bsum = fabs(Xt_b_data[2].im);
                  if (brm > bsum) {
                    HeadingCandA_tmp = Xt_b_data[2].im / Xt_b_data[2].re;
                    bsum = Xt_b_data[2].re + HeadingCandA_tmp * Xt_b_data[2].im;
                    b_z[1].re = (ar + HeadingCandA_tmp * ai) / bsum;
                    b_z[1].im = (ai - HeadingCandA_tmp * ar) / bsum;
                  } else if (bsum == brm) {
                    if (Xt_b_data[2].re > 0.0) {
                      HeadingCandA_tmp = 0.5;
                    } else {
                      HeadingCandA_tmp = -0.5;
                    }
                    if (Xt_b_data[2].im > 0.0) {
                      bsum = 0.5;
                    } else {
                      bsum = -0.5;
                    }
                    b_z[1].re = (ar * HeadingCandA_tmp + ai * bsum) / brm;
                    b_z[1].im = (ai * HeadingCandA_tmp - ar * bsum) / brm;
                  } else {
                    HeadingCandA_tmp = Xt_b_data[2].re / Xt_b_data[2].im;
                    bsum = Xt_b_data[2].im + HeadingCandA_tmp * Xt_b_data[2].re;
                    b_z[1].re = (HeadingCandA_tmp * ar + ai) / bsum;
                    b_z[1].im = (HeadingCandA_tmp * ai - ar) / bsum;
                  }
                }
                ar = b_z[0].re + b_z[1].re;
                ai = b_z[0].im + b_z[1].im;
                if (ai == 0.0) {
                  HeadingCandA.re = ar / 2.0;
                  HeadingCandA.im = 0.0;
                } else if (ar == 0.0) {
                  HeadingCandA.re = 0.0;
                  HeadingCandA.im = ai / 2.0;
                } else {
                  HeadingCandA.re = ar / 2.0;
                  HeadingCandA.im = ai / 2.0;
                }
              } else if (Temp_tmp && b_Temp_tmp) {
                ar = Temp[0].re - Temp[1].re;
                ai = Temp[0].im - Temp[1].im;
                br = Xt_b_data[0].re - Xt_b_data[1].re;
                bi = Xt_b_data[0].im - Xt_b_data[1].im;
                if (bi == 0.0) {
                  if (ai == 0.0) {
                    HeadingCandA.re = ar / br;
                    HeadingCandA.im = 0.0;
                  } else if (ar == 0.0) {
                    HeadingCandA.re = 0.0;
                    HeadingCandA.im = ai / br;
                  } else {
                    HeadingCandA.re = ar / br;
                    HeadingCandA.im = ai / br;
                  }
                } else if (br == 0.0) {
                  if (ar == 0.0) {
                    HeadingCandA.re = ai / bi;
                    HeadingCandA.im = 0.0;
                  } else if (ai == 0.0) {
                    HeadingCandA.re = 0.0;
                    HeadingCandA.im = -(ar / bi);
                  } else {
                    HeadingCandA.re = ai / bi;
                    HeadingCandA.im = -(ar / bi);
                  }
                } else {
                  brm = fabs(br);
                  bsum = fabs(bi);
                  if (brm > bsum) {
                    HeadingCandA_tmp = bi / br;
                    bsum = br + HeadingCandA_tmp * bi;
                    HeadingCandA.re = (ar + HeadingCandA_tmp * ai) / bsum;
                    HeadingCandA.im = (ai - HeadingCandA_tmp * ar) / bsum;
                  } else if (bsum == brm) {
                    if (br > 0.0) {
                      HeadingCandA_tmp = 0.5;
                    } else {
                      HeadingCandA_tmp = -0.5;
                    }
                    if (bi > 0.0) {
                      bsum = 0.5;
                    } else {
                      bsum = -0.5;
                    }
                    HeadingCandA.re = (ar * HeadingCandA_tmp + ai * bsum) / brm;
                    HeadingCandA.im = (ai * HeadingCandA_tmp - ar * bsum) / brm;
                  } else {
                    HeadingCandA_tmp = br / bi;
                    bsum = bi + HeadingCandA_tmp * br;
                    HeadingCandA.re = (HeadingCandA_tmp * ar + ai) / bsum;
                    HeadingCandA.im = (HeadingCandA_tmp * ai - ar) / bsum;
                  }
                }
                ar = Temp[0].re + Temp[1].re;
                ai = Temp[0].im + Temp[1].im;
                if (ai == 0.0) {
                  brm = ar / 2.0;
                  bi = 0.0;
                } else if (ar == 0.0) {
                  brm = 0.0;
                  bi = ai / 2.0;
                } else {
                  brm = ar / 2.0;
                  bi = ai / 2.0;
                }
                ar = Xt_b_data[0].re + Xt_b_data[1].re;
                ai = Xt_b_data[0].im + Xt_b_data[1].im;
                if (ai == 0.0) {
                  PosUWB2_tmp = ar / 2.0;
                  HeadingCandA_tmp = 0.0;
                } else if (ar == 0.0) {
                  PosUWB2_tmp = 0.0;
                  HeadingCandA_tmp = ai / 2.0;
                } else {
                  PosUWB2_tmp = ar / 2.0;
                  HeadingCandA_tmp = ai / 2.0;
                }
                bsum = PosUWB2_tmp * HeadingCandA.re -
                       HeadingCandA_tmp * HeadingCandA.im;
                HeadingCandA_tmp = PosUWB2_tmp * HeadingCandA.im +
                                   HeadingCandA_tmp * HeadingCandA.re;
                br = rt_hypotd_snf(HeadingCandA.re, HeadingCandA.im);
                if (HeadingCandA_tmp == 0.0) {
                  PosUWB2_tmp = bsum / br;
                  HeadingCandA_tmp = 0.0;
                } else if (bsum == 0.0) {
                  PosUWB2_tmp = 0.0;
                  HeadingCandA_tmp /= br;
                } else {
                  PosUWB2_tmp = bsum / br;
                  HeadingCandA_tmp /= br;
                }
                CenterCandA.re = brm - PosUWB2_tmp;
                CenterCandA.im = bi - HeadingCandA_tmp;
              } else if (b_Temp_tmp) {
                PosUWB2_tmp = b_PosUWB2[3].f1.data[nn].re;
                b_PosUWB2_tmp = b_PosUWB2[3].f1.data[nn].im;
                if ((PosUWB2_tmp != 0.0) || (b_PosUWB2_tmp != 0.0)) {
                  ar = Temp[1].re - PosUWB2_tmp;
                  ai = Temp[1].im - b_PosUWB2_tmp;
                  br = Xt_b_data[1].re - Xt_b_data[3].re;
                  bi = Xt_b_data[1].im - Xt_b_data[3].im;
                  if (bi == 0.0) {
                    if (ai == 0.0) {
                      HeadingCandA.re = ar / br;
                      HeadingCandA.im = 0.0;
                    } else if (ar == 0.0) {
                      HeadingCandA.re = 0.0;
                      HeadingCandA.im = ai / br;
                    } else {
                      HeadingCandA.re = ar / br;
                      HeadingCandA.im = ai / br;
                    }
                  } else if (br == 0.0) {
                    if (ar == 0.0) {
                      HeadingCandA.re = ai / bi;
                      HeadingCandA.im = 0.0;
                    } else if (ai == 0.0) {
                      HeadingCandA.re = 0.0;
                      HeadingCandA.im = -(ar / bi);
                    } else {
                      HeadingCandA.re = ai / bi;
                      HeadingCandA.im = -(ar / bi);
                    }
                  } else {
                    brm = fabs(br);
                    bsum = fabs(bi);
                    if (brm > bsum) {
                      HeadingCandA_tmp = bi / br;
                      bsum = br + HeadingCandA_tmp * bi;
                      HeadingCandA.re = (ar + HeadingCandA_tmp * ai) / bsum;
                      HeadingCandA.im = (ai - HeadingCandA_tmp * ar) / bsum;
                    } else if (bsum == brm) {
                      if (br > 0.0) {
                        HeadingCandA_tmp = 0.5;
                      } else {
                        HeadingCandA_tmp = -0.5;
                      }
                      if (bi > 0.0) {
                        bsum = 0.5;
                      } else {
                        bsum = -0.5;
                      }
                      HeadingCandA.re =
                          (ar * HeadingCandA_tmp + ai * bsum) / brm;
                      HeadingCandA.im =
                          (ai * HeadingCandA_tmp - ar * bsum) / brm;
                    } else {
                      HeadingCandA_tmp = br / bi;
                      bsum = bi + HeadingCandA_tmp * br;
                      HeadingCandA.re = (HeadingCandA_tmp * ar + ai) / bsum;
                      HeadingCandA.im = (HeadingCandA_tmp * ai - ar) / bsum;
                    }
                  }
                  ar = Temp[1].re + PosUWB2_tmp;
                  ai = Temp[1].im + b_PosUWB2_tmp;
                  if (ai == 0.0) {
                    brm = ar / 2.0;
                    bi = 0.0;
                  } else if (ar == 0.0) {
                    brm = 0.0;
                    bi = ai / 2.0;
                  } else {
                    brm = ar / 2.0;
                    bi = ai / 2.0;
                  }
                  ar = Xt_b_data[1].re + Xt_b_data[3].re;
                  ai = Xt_b_data[1].im + Xt_b_data[3].im;
                  if (ai == 0.0) {
                    PosUWB2_tmp = ar / 2.0;
                    HeadingCandA_tmp = 0.0;
                  } else if (ar == 0.0) {
                    PosUWB2_tmp = 0.0;
                    HeadingCandA_tmp = ai / 2.0;
                  } else {
                    PosUWB2_tmp = ar / 2.0;
                    HeadingCandA_tmp = ai / 2.0;
                  }
                  bsum = PosUWB2_tmp * HeadingCandA.re -
                         HeadingCandA_tmp * HeadingCandA.im;
                  HeadingCandA_tmp = PosUWB2_tmp * HeadingCandA.im +
                                     HeadingCandA_tmp * HeadingCandA.re;
                  br = rt_hypotd_snf(HeadingCandA.re, HeadingCandA.im);
                  if (HeadingCandA_tmp == 0.0) {
                    PosUWB2_tmp = bsum / br;
                    HeadingCandA_tmp = 0.0;
                  } else if (bsum == 0.0) {
                    PosUWB2_tmp = 0.0;
                    HeadingCandA_tmp /= br;
                  } else {
                    PosUWB2_tmp = bsum / br;
                    HeadingCandA_tmp /= br;
                  }
                  CenterCandA.re = brm - PosUWB2_tmp;
                  CenterCandA.im = bi - HeadingCandA_tmp;
                } else {
                  guard2 = true;
                }
              } else {
                guard2 = true;
              }
            }
            if (guard2) {
              b_Temp_tmp = ((Temp[2].re != 0.0) || (Temp[2].im != 0.0));
              if (b_Temp_tmp) {
                PosUWB2_tmp = b_PosUWB2[3].f1.data[nn].re;
                b_PosUWB2_tmp = b_PosUWB2[3].f1.data[nn].im;
                if ((PosUWB2_tmp != 0.0) || (b_PosUWB2_tmp != 0.0)) {
                  ar = Temp[2].re - PosUWB2_tmp;
                  ai = Temp[2].im - b_PosUWB2_tmp;
                  br = Xt_b_data[2].re - Xt_b_data[3].re;
                  bi = Xt_b_data[2].im - Xt_b_data[3].im;
                  if (bi == 0.0) {
                    if (ai == 0.0) {
                      HeadingCandA.re = ar / br;
                      HeadingCandA.im = 0.0;
                    } else if (ar == 0.0) {
                      HeadingCandA.re = 0.0;
                      HeadingCandA.im = ai / br;
                    } else {
                      HeadingCandA.re = ar / br;
                      HeadingCandA.im = ai / br;
                    }
                  } else if (br == 0.0) {
                    if (ar == 0.0) {
                      HeadingCandA.re = ai / bi;
                      HeadingCandA.im = 0.0;
                    } else if (ai == 0.0) {
                      HeadingCandA.re = 0.0;
                      HeadingCandA.im = -(ar / bi);
                    } else {
                      HeadingCandA.re = ai / bi;
                      HeadingCandA.im = -(ar / bi);
                    }
                  } else {
                    brm = fabs(br);
                    bsum = fabs(bi);
                    if (brm > bsum) {
                      HeadingCandA_tmp = bi / br;
                      bsum = br + HeadingCandA_tmp * bi;
                      HeadingCandA.re = (ar + HeadingCandA_tmp * ai) / bsum;
                      HeadingCandA.im = (ai - HeadingCandA_tmp * ar) / bsum;
                    } else if (bsum == brm) {
                      if (br > 0.0) {
                        HeadingCandA_tmp = 0.5;
                      } else {
                        HeadingCandA_tmp = -0.5;
                      }
                      if (bi > 0.0) {
                        bsum = 0.5;
                      } else {
                        bsum = -0.5;
                      }
                      HeadingCandA.re =
                          (ar * HeadingCandA_tmp + ai * bsum) / brm;
                      HeadingCandA.im =
                          (ai * HeadingCandA_tmp - ar * bsum) / brm;
                    } else {
                      HeadingCandA_tmp = br / bi;
                      bsum = bi + HeadingCandA_tmp * br;
                      HeadingCandA.re = (HeadingCandA_tmp * ar + ai) / bsum;
                      HeadingCandA.im = (HeadingCandA_tmp * ai - ar) / bsum;
                    }
                  }
                  ar = Temp[2].re + PosUWB2_tmp;
                  ai = Temp[2].im + b_PosUWB2_tmp;
                  if (ai == 0.0) {
                    brm = ar / 2.0;
                    bi = 0.0;
                  } else if (ar == 0.0) {
                    brm = 0.0;
                    bi = ai / 2.0;
                  } else {
                    brm = ar / 2.0;
                    bi = ai / 2.0;
                  }
                  ar = Xt_b_data[2].re + Xt_b_data[3].re;
                  ai = Xt_b_data[2].im + Xt_b_data[3].im;
                  if (ai == 0.0) {
                    PosUWB2_tmp = ar / 2.0;
                    HeadingCandA_tmp = 0.0;
                  } else if (ar == 0.0) {
                    PosUWB2_tmp = 0.0;
                    HeadingCandA_tmp = ai / 2.0;
                  } else {
                    PosUWB2_tmp = ar / 2.0;
                    HeadingCandA_tmp = ai / 2.0;
                  }
                  bsum = PosUWB2_tmp * HeadingCandA.re -
                         HeadingCandA_tmp * HeadingCandA.im;
                  HeadingCandA_tmp = PosUWB2_tmp * HeadingCandA.im +
                                     HeadingCandA_tmp * HeadingCandA.re;
                  br = rt_hypotd_snf(HeadingCandA.re, HeadingCandA.im);
                  if (HeadingCandA_tmp == 0.0) {
                    PosUWB2_tmp = bsum / br;
                    HeadingCandA_tmp = 0.0;
                  } else if (bsum == 0.0) {
                    PosUWB2_tmp = 0.0;
                    HeadingCandA_tmp /= br;
                  } else {
                    PosUWB2_tmp = bsum / br;
                    HeadingCandA_tmp /= br;
                  }
                  CenterCandA.re = brm - PosUWB2_tmp;
                  CenterCandA.im = bi - HeadingCandA_tmp;
                } else {
                  guard1 = true;
                }
              } else {
                guard1 = true;
              }
            }
            if (guard1) {
              if (b_Temp_tmp && Temp_tmp) {
                ar = Temp[2].re - Temp[0].re;
                ai = Temp[2].im - Temp[0].im;
                br = Xt_b_data[2].re - Xt_b_data[0].re;
                bi = Xt_b_data[2].im - Xt_b_data[0].im;
                if (bi == 0.0) {
                  if (ai == 0.0) {
                    HeadingCandA.re = ar / br;
                    HeadingCandA.im = 0.0;
                  } else if (ar == 0.0) {
                    HeadingCandA.re = 0.0;
                    HeadingCandA.im = ai / br;
                  } else {
                    HeadingCandA.re = ar / br;
                    HeadingCandA.im = ai / br;
                  }
                } else if (br == 0.0) {
                  if (ar == 0.0) {
                    HeadingCandA.re = ai / bi;
                    HeadingCandA.im = 0.0;
                  } else if (ai == 0.0) {
                    HeadingCandA.re = 0.0;
                    HeadingCandA.im = -(ar / bi);
                  } else {
                    HeadingCandA.re = ai / bi;
                    HeadingCandA.im = -(ar / bi);
                  }
                } else {
                  brm = fabs(br);
                  bsum = fabs(bi);
                  if (brm > bsum) {
                    HeadingCandA_tmp = bi / br;
                    bsum = br + HeadingCandA_tmp * bi;
                    HeadingCandA.re = (ar + HeadingCandA_tmp * ai) / bsum;
                    HeadingCandA.im = (ai - HeadingCandA_tmp * ar) / bsum;
                  } else if (bsum == brm) {
                    if (br > 0.0) {
                      HeadingCandA_tmp = 0.5;
                    } else {
                      HeadingCandA_tmp = -0.5;
                    }
                    if (bi > 0.0) {
                      bsum = 0.5;
                    } else {
                      bsum = -0.5;
                    }
                    HeadingCandA.re = (ar * HeadingCandA_tmp + ai * bsum) / brm;
                    HeadingCandA.im = (ai * HeadingCandA_tmp - ar * bsum) / brm;
                  } else {
                    HeadingCandA_tmp = br / bi;
                    bsum = bi + HeadingCandA_tmp * br;
                    HeadingCandA.re = (HeadingCandA_tmp * ar + ai) / bsum;
                    HeadingCandA.im = (HeadingCandA_tmp * ai - ar) / bsum;
                  }
                }
                ar = Temp[0].re + Temp[2].re;
                ai = Temp[0].im + Temp[2].im;
                if (ai == 0.0) {
                  brm = ar / 2.0;
                  bi = 0.0;
                } else if (ar == 0.0) {
                  brm = 0.0;
                  bi = ai / 2.0;
                } else {
                  brm = ar / 2.0;
                  bi = ai / 2.0;
                }
                ar = Xt_b_data[0].re + Xt_b_data[2].re;
                ai = Xt_b_data[0].im + Xt_b_data[2].im;
                if (ai == 0.0) {
                  PosUWB2_tmp = ar / 2.0;
                  HeadingCandA_tmp = 0.0;
                } else if (ar == 0.0) {
                  PosUWB2_tmp = 0.0;
                  HeadingCandA_tmp = ai / 2.0;
                } else {
                  PosUWB2_tmp = ar / 2.0;
                  HeadingCandA_tmp = ai / 2.0;
                }
                bsum = PosUWB2_tmp * HeadingCandA.re -
                       HeadingCandA_tmp * HeadingCandA.im;
                HeadingCandA_tmp = PosUWB2_tmp * HeadingCandA.im +
                                   HeadingCandA_tmp * HeadingCandA.re;
                br = rt_hypotd_snf(HeadingCandA.re, HeadingCandA.im);
                if (HeadingCandA_tmp == 0.0) {
                  PosUWB2_tmp = bsum / br;
                  HeadingCandA_tmp = 0.0;
                } else if (bsum == 0.0) {
                  PosUWB2_tmp = 0.0;
                  HeadingCandA_tmp /= br;
                } else {
                  PosUWB2_tmp = bsum / br;
                  HeadingCandA_tmp /= br;
                }
                CenterCandA.re = brm - PosUWB2_tmp;
                CenterCandA.im = bi - HeadingCandA_tmp;
              } else {
                HeadingCandA_tmp =
                    ((Temp[0].re + Temp[1].re) + Temp[2].re) + Temp[3].re;
                bsum = ((Temp[0].im + Temp[1].im) + Temp[2].im) + Temp[3].im;
                if (bsum == 0.0) {
                  CenterCandA.re = HeadingCandA_tmp / 4.0;
                  CenterCandA.im = 0.0;
                } else if (HeadingCandA_tmp == 0.0) {
                  CenterCandA.re = 0.0;
                  CenterCandA.im = bsum / 4.0;
                } else {
                  CenterCandA.re = HeadingCandA_tmp / 4.0;
                  CenterCandA.im = bsum / 4.0;
                }
                if (Xt_b_size[1] == 4) {
                  ar = Temp[0].re - CenterCandA.re;
                  ai = Temp[0].im - CenterCandA.im;
                  if (Xt_b_data[0].im == 0.0) {
                    if (ai == 0.0) {
                      z[0].re = ar / Xt_b_data[0].re;
                      z[0].im = 0.0;
                    } else if (ar == 0.0) {
                      z[0].re = 0.0;
                      z[0].im = ai / Xt_b_data[0].re;
                    } else {
                      z[0].re = ar / Xt_b_data[0].re;
                      z[0].im = ai / Xt_b_data[0].re;
                    }
                  } else if (Xt_b_data[0].re == 0.0) {
                    if (ar == 0.0) {
                      z[0].re = ai / Xt_b_data[0].im;
                      z[0].im = 0.0;
                    } else if (ai == 0.0) {
                      z[0].re = 0.0;
                      z[0].im = -(ar / Xt_b_data[0].im);
                    } else {
                      z[0].re = ai / Xt_b_data[0].im;
                      z[0].im = -(ar / Xt_b_data[0].im);
                    }
                  } else {
                    brm = fabs(Xt_b_data[0].re);
                    bsum = fabs(Xt_b_data[0].im);
                    if (brm > bsum) {
                      HeadingCandA_tmp = Xt_b_data[0].im / Xt_b_data[0].re;
                      bsum =
                          Xt_b_data[0].re + HeadingCandA_tmp * Xt_b_data[0].im;
                      z[0].re = (ar + HeadingCandA_tmp * ai) / bsum;
                      z[0].im = (ai - HeadingCandA_tmp * ar) / bsum;
                    } else if (bsum == brm) {
                      if (Xt_b_data[0].re > 0.0) {
                        HeadingCandA_tmp = 0.5;
                      } else {
                        HeadingCandA_tmp = -0.5;
                      }
                      if (Xt_b_data[0].im > 0.0) {
                        bsum = 0.5;
                      } else {
                        bsum = -0.5;
                      }
                      z[0].re = (ar * HeadingCandA_tmp + ai * bsum) / brm;
                      z[0].im = (ai * HeadingCandA_tmp - ar * bsum) / brm;
                    } else {
                      HeadingCandA_tmp = Xt_b_data[0].re / Xt_b_data[0].im;
                      bsum =
                          Xt_b_data[0].im + HeadingCandA_tmp * Xt_b_data[0].re;
                      z[0].re = (HeadingCandA_tmp * ar + ai) / bsum;
                      z[0].im = (HeadingCandA_tmp * ai - ar) / bsum;
                    }
                  }
                  ar = Temp[1].re - CenterCandA.re;
                  ai = Temp[1].im - CenterCandA.im;
                  if (Xt_b_data[1].im == 0.0) {
                    if (ai == 0.0) {
                      z[1].re = ar / Xt_b_data[1].re;
                      z[1].im = 0.0;
                    } else if (ar == 0.0) {
                      z[1].re = 0.0;
                      z[1].im = ai / Xt_b_data[1].re;
                    } else {
                      z[1].re = ar / Xt_b_data[1].re;
                      z[1].im = ai / Xt_b_data[1].re;
                    }
                  } else if (Xt_b_data[1].re == 0.0) {
                    if (ar == 0.0) {
                      z[1].re = ai / Xt_b_data[1].im;
                      z[1].im = 0.0;
                    } else if (ai == 0.0) {
                      z[1].re = 0.0;
                      z[1].im = -(ar / Xt_b_data[1].im);
                    } else {
                      z[1].re = ai / Xt_b_data[1].im;
                      z[1].im = -(ar / Xt_b_data[1].im);
                    }
                  } else {
                    brm = fabs(Xt_b_data[1].re);
                    bsum = fabs(Xt_b_data[1].im);
                    if (brm > bsum) {
                      HeadingCandA_tmp = Xt_b_data[1].im / Xt_b_data[1].re;
                      bsum =
                          Xt_b_data[1].re + HeadingCandA_tmp * Xt_b_data[1].im;
                      z[1].re = (ar + HeadingCandA_tmp * ai) / bsum;
                      z[1].im = (ai - HeadingCandA_tmp * ar) / bsum;
                    } else if (bsum == brm) {
                      if (Xt_b_data[1].re > 0.0) {
                        HeadingCandA_tmp = 0.5;
                      } else {
                        HeadingCandA_tmp = -0.5;
                      }
                      if (Xt_b_data[1].im > 0.0) {
                        bsum = 0.5;
                      } else {
                        bsum = -0.5;
                      }
                      z[1].re = (ar * HeadingCandA_tmp + ai * bsum) / brm;
                      z[1].im = (ai * HeadingCandA_tmp - ar * bsum) / brm;
                    } else {
                      HeadingCandA_tmp = Xt_b_data[1].re / Xt_b_data[1].im;
                      bsum =
                          Xt_b_data[1].im + HeadingCandA_tmp * Xt_b_data[1].re;
                      z[1].re = (HeadingCandA_tmp * ar + ai) / bsum;
                      z[1].im = (HeadingCandA_tmp * ai - ar) / bsum;
                    }
                  }
                  ar = Temp[2].re - CenterCandA.re;
                  ai = Temp[2].im - CenterCandA.im;
                  if (Xt_b_data[2].im == 0.0) {
                    if (ai == 0.0) {
                      z[2].re = ar / Xt_b_data[2].re;
                      z[2].im = 0.0;
                    } else if (ar == 0.0) {
                      z[2].re = 0.0;
                      z[2].im = ai / Xt_b_data[2].re;
                    } else {
                      z[2].re = ar / Xt_b_data[2].re;
                      z[2].im = ai / Xt_b_data[2].re;
                    }
                  } else if (Xt_b_data[2].re == 0.0) {
                    if (ar == 0.0) {
                      z[2].re = ai / Xt_b_data[2].im;
                      z[2].im = 0.0;
                    } else if (ai == 0.0) {
                      z[2].re = 0.0;
                      z[2].im = -(ar / Xt_b_data[2].im);
                    } else {
                      z[2].re = ai / Xt_b_data[2].im;
                      z[2].im = -(ar / Xt_b_data[2].im);
                    }
                  } else {
                    brm = fabs(Xt_b_data[2].re);
                    bsum = fabs(Xt_b_data[2].im);
                    if (brm > bsum) {
                      HeadingCandA_tmp = Xt_b_data[2].im / Xt_b_data[2].re;
                      bsum =
                          Xt_b_data[2].re + HeadingCandA_tmp * Xt_b_data[2].im;
                      z[2].re = (ar + HeadingCandA_tmp * ai) / bsum;
                      z[2].im = (ai - HeadingCandA_tmp * ar) / bsum;
                    } else if (bsum == brm) {
                      if (Xt_b_data[2].re > 0.0) {
                        HeadingCandA_tmp = 0.5;
                      } else {
                        HeadingCandA_tmp = -0.5;
                      }
                      if (Xt_b_data[2].im > 0.0) {
                        bsum = 0.5;
                      } else {
                        bsum = -0.5;
                      }
                      z[2].re = (ar * HeadingCandA_tmp + ai * bsum) / brm;
                      z[2].im = (ai * HeadingCandA_tmp - ar * bsum) / brm;
                    } else {
                      HeadingCandA_tmp = Xt_b_data[2].re / Xt_b_data[2].im;
                      bsum =
                          Xt_b_data[2].im + HeadingCandA_tmp * Xt_b_data[2].re;
                      z[2].re = (HeadingCandA_tmp * ar + ai) / bsum;
                      z[2].im = (HeadingCandA_tmp * ai - ar) / bsum;
                    }
                  }
                  ar = Temp[3].re - CenterCandA.re;
                  ai = Temp[3].im - CenterCandA.im;
                  if (Xt_b_data[3].im == 0.0) {
                    if (ai == 0.0) {
                      z[3].re = ar / Xt_b_data[3].re;
                      z[3].im = 0.0;
                    } else if (ar == 0.0) {
                      z[3].re = 0.0;
                      z[3].im = ai / Xt_b_data[3].re;
                    } else {
                      z[3].re = ar / Xt_b_data[3].re;
                      z[3].im = ai / Xt_b_data[3].re;
                    }
                  } else if (Xt_b_data[3].re == 0.0) {
                    if (ar == 0.0) {
                      z[3].re = ai / Xt_b_data[3].im;
                      z[3].im = 0.0;
                    } else if (ai == 0.0) {
                      z[3].re = 0.0;
                      z[3].im = -(ar / Xt_b_data[3].im);
                    } else {
                      z[3].re = ai / Xt_b_data[3].im;
                      z[3].im = -(ar / Xt_b_data[3].im);
                    }
                  } else {
                    brm = fabs(Xt_b_data[3].re);
                    bsum = fabs(Xt_b_data[3].im);
                    if (brm > bsum) {
                      HeadingCandA_tmp = Xt_b_data[3].im / Xt_b_data[3].re;
                      bsum =
                          Xt_b_data[3].re + HeadingCandA_tmp * Xt_b_data[3].im;
                      z[3].re = (ar + HeadingCandA_tmp * ai) / bsum;
                      z[3].im = (ai - HeadingCandA_tmp * ar) / bsum;
                    } else if (bsum == brm) {
                      if (Xt_b_data[3].re > 0.0) {
                        HeadingCandA_tmp = 0.5;
                      } else {
                        HeadingCandA_tmp = -0.5;
                      }
                      if (Xt_b_data[3].im > 0.0) {
                        bsum = 0.5;
                      } else {
                        bsum = -0.5;
                      }
                      z[3].re = (ar * HeadingCandA_tmp + ai * bsum) / brm;
                      z[3].im = (ai * HeadingCandA_tmp - ar * bsum) / brm;
                    } else {
                      HeadingCandA_tmp = Xt_b_data[3].re / Xt_b_data[3].im;
                      bsum =
                          Xt_b_data[3].im + HeadingCandA_tmp * Xt_b_data[3].re;
                      z[3].re = (HeadingCandA_tmp * ar + ai) / bsum;
                      z[3].im = (HeadingCandA_tmp * ai - ar) / bsum;
                    }
                  }
                } else {
                  binary_expand_op_3(z, Temp, CenterCandA, Xt_b_data,
                                     Xt_b_size);
                }
                HeadingCandA_tmp = ((z[0].re + z[1].re) + z[2].re) + z[3].re;
                bsum = ((z[0].im + z[1].im) + z[2].im) + z[3].im;
                if (bsum == 0.0) {
                  HeadingCandA_tmp /= 4.0;
                  bsum = 0.0;
                } else if (HeadingCandA_tmp == 0.0) {
                  HeadingCandA_tmp = 0.0;
                  bsum /= 4.0;
                } else {
                  HeadingCandA_tmp /= 4.0;
                  bsum /= 4.0;
                }
                HeadingCandA.re = HeadingCandA_tmp;
                HeadingCandA.im = bsum;
              }
            }
          }
          brm = rt_hypotd_snf(HeadingCandA.re, HeadingCandA.im);
          HeadingCandA_tmp = fabs(brm - 1.0);
          if (HeadingCandA_tmp <= 1.0) {
            boolean_T exitg1;
            PosUWB2_tmp = Xt_b_data[0].re * HeadingCandA.re -
                          Xt_b_data[0].im * HeadingCandA.im;
            HeadingCandA_tmp = Xt_b_data[0].re * HeadingCandA.im +
                               Xt_b_data[0].im * HeadingCandA.re;
            if (HeadingCandA_tmp == 0.0) {
              PosUWB2_tmp /= brm;
              HeadingCandA_tmp = 0.0;
            } else if (PosUWB2_tmp == 0.0) {
              PosUWB2_tmp = 0.0;
              HeadingCandA_tmp /= brm;
            } else {
              PosUWB2_tmp /= brm;
              HeadingCandA_tmp /= brm;
            }
            TagCandA[ss].re = CenterCandA.re + PosUWB2_tmp;
            TagCandA[ss].im = CenterCandA.im + HeadingCandA_tmp;
            PosUWB2_tmp = Xt_b_data[1].re * HeadingCandA.re -
                          Xt_b_data[1].im * HeadingCandA.im;
            HeadingCandA_tmp = Xt_b_data[1].re * HeadingCandA.im +
                               Xt_b_data[1].im * HeadingCandA.re;
            if (HeadingCandA_tmp == 0.0) {
              PosUWB2_tmp /= brm;
              HeadingCandA_tmp = 0.0;
            } else if (PosUWB2_tmp == 0.0) {
              PosUWB2_tmp = 0.0;
              HeadingCandA_tmp /= brm;
            } else {
              PosUWB2_tmp /= brm;
              HeadingCandA_tmp /= brm;
            }
            TagCandA[ss + 20000].re = CenterCandA.re + PosUWB2_tmp;
            TagCandA[ss + 20000].im = CenterCandA.im + HeadingCandA_tmp;
            PosUWB2_tmp = Xt_b_data[2].re * HeadingCandA.re -
                          Xt_b_data[2].im * HeadingCandA.im;
            HeadingCandA_tmp = Xt_b_data[2].re * HeadingCandA.im +
                               Xt_b_data[2].im * HeadingCandA.re;
            if (HeadingCandA_tmp == 0.0) {
              PosUWB2_tmp /= brm;
              HeadingCandA_tmp = 0.0;
            } else if (PosUWB2_tmp == 0.0) {
              PosUWB2_tmp = 0.0;
              HeadingCandA_tmp /= brm;
            } else {
              PosUWB2_tmp /= brm;
              HeadingCandA_tmp /= brm;
            }
            TagCandA[ss + 40000].re = CenterCandA.re + PosUWB2_tmp;
            TagCandA[ss + 40000].im = CenterCandA.im + HeadingCandA_tmp;
            PosUWB2_tmp = Xt_b_data[3].re * HeadingCandA.re -
                          Xt_b_data[3].im * HeadingCandA.im;
            HeadingCandA_tmp = Xt_b_data[3].re * HeadingCandA.im +
                               Xt_b_data[3].im * HeadingCandA.re;
            if (HeadingCandA_tmp == 0.0) {
              PosUWB2_tmp /= brm;
              HeadingCandA_tmp = 0.0;
            } else if (PosUWB2_tmp == 0.0) {
              PosUWB2_tmp = 0.0;
              HeadingCandA_tmp /= brm;
            } else {
              PosUWB2_tmp /= brm;
              HeadingCandA_tmp /= brm;
            }
            TagCandA[ss + 60000].re = CenterCandA.re + PosUWB2_tmp;
            TagCandA[ss + 60000].im = CenterCandA.im + HeadingCandA_tmp;
            k = x->size[0] * x->size[1];
            x->size[0] = 4;
            lastBlockLength = Xain->size[1];
            x->size[1] = Xain->size[1];
            emxEnsureCapacity_creal_T(x, k);
            sortedCenter_data = x->data;
            for (k = 0; k < lastBlockLength; k++) {
              HeadingCandA_tmp = Xain_data[k].re;
              sortedCenter_data[4 * k].re = TagCandA[ss].re - HeadingCandA_tmp;
              bsum = Xain_data[k].im;
              sortedCenter_data[4 * k].im = TagCandA[ss].im - bsum;
              k0 = 4 * k + 1;
              sortedCenter_data[k0].re =
                  TagCandA[ss + 20000].re - HeadingCandA_tmp;
              sortedCenter_data[k0].im = TagCandA[ss + 20000].im - bsum;
              k0 = 4 * k + 2;
              sortedCenter_data[k0].re =
                  TagCandA[ss + 40000].re - HeadingCandA_tmp;
              sortedCenter_data[k0].im = TagCandA[ss + 40000].im - bsum;
              k0 = 4 * k + 3;
              sortedCenter_data[k0].re =
                  TagCandA[ss + 60000].re - HeadingCandA_tmp;
              sortedCenter_data[k0].im = TagCandA[ss + 60000].im - bsum;
            }
            nx = x->size[1] << 2;
            k = TempA->size[0] * TempA->size[1];
            TempA->size[0] = 4;
            TempA->size[1] = x->size[1];
            emxEnsureCapacity_real_T(TempA, k);
            TempA_data = TempA->data;
            for (k = 0; k < nx; k++) {
              TempA_data[k] = rt_hypotd_snf(sortedCenter_data[k].re,
                                            sortedCenter_data[k].im);
            }
            lastBlockLength = b_DistMap->size[1];
            if (b_DistMap->size[1] == TempA->size[1]) {
              k = TempA->size[0] * TempA->size[1];
              TempA->size[0] = 4;
              TempA->size[1] = b_DistMap->size[1];
              emxEnsureCapacity_real_T(TempA, k);
              TempA_data = TempA->data;
              for (k = 0; k < lastBlockLength; k++) {
                TempA_data[4 * k] =
                    DistMap_data[4 * k + 4 * b_DistMap->size[1] * 99] -
                    TempA_data[4 * k];
                k0 = 4 * k + 1;
                TempA_data[k0] =
                    DistMap_data[(4 * k + 4 * b_DistMap->size[1] * 99) + 1] -
                    TempA_data[k0];
                k0 = 4 * k + 2;
                TempA_data[k0] =
                    DistMap_data[(4 * k + 4 * b_DistMap->size[1] * 99) + 2] -
                    TempA_data[k0];
                k0 = 4 * k + 3;
                TempA_data[k0] =
                    DistMap_data[(4 * k + 4 * b_DistMap->size[1] * 99) + 3] -
                    TempA_data[k0];
              }
            } else {
              binary_expand_op_2(TempA, b_DistMap);
              TempA_data = TempA->data;
            }
            k0 = TempA->size[1] << 2;
            k = DistErr->size[0] * DistErr->size[1];
            DistErr->size[0] = 4;
            DistErr->size[1] = TempA->size[1];
            emxEnsureCapacity_real_T(DistErr, k);
            DistErr_data = DistErr->data;
            for (k = 0; k < k0; k++) {
              DistErr_data[k] = fabs(TempA_data[k]);
            }
            k = b_x->size[0];
            b_x->size[0] = k0;
            emxEnsureCapacity_boolean_T(b_x, k);
            x_data = b_x->data;
            for (k = 0; k < k0; k++) {
              x_data[k] = (DistErr_data[k] < 1.0);
            }
            nx = b_x->size[0];
            idx = 0;
            k = b_i->size[0];
            b_i->size[0] = b_x->size[0];
            emxEnsureCapacity_int32_T(b_i, k);
            i_data = b_i->data;
            k0 = 0;
            exitg1 = false;
            while ((!exitg1) && (k0 <= nx - 1)) {
              if (x_data[k0]) {
                idx++;
                i_data[idx - 1] = k0 + 1;
                if (idx >= nx) {
                  exitg1 = true;
                } else {
                  k0++;
                }
              } else {
                k0++;
              }
            }
            if (b_x->size[0] == 1) {
              if (idx == 0) {
                b_i->size[0] = 0;
              }
            } else {
              k = b_i->size[0];
              if (idx < 1) {
                b_i->size[0] = 0;
              } else {
                b_i->size[0] = idx;
              }
              emxEnsureCapacity_int32_T(b_i, k);
            }
            LessThan1m1[ss] = (unsigned int)b_i->size[0];
            CenterCands[ss] = CenterCandA;
            HeadingCands[ss] = HeadingCandA;
            ss++;
          } else if (HeadingCandA_tmp < PrevAbsHeadingCandA) {
            PrevAbsHeadingCandA = HeadingCandA_tmp;
            *PosHH = CenterCandA;
            PrevHeadingCandA_re = HeadingCandA.re;
            PrevHeadingCandA_im = HeadingCandA.im;
          }
        }
      }
    }
  }
  emxFree_int32_T(&b_i);
  emxFree_creal_T(&x);
  emxFree_real_T(&DistErr);
  emxInit_creal_T(&sortedCenter);
  emxInit_creal_T(&sortedHeading);
  emxInit_int32_T(&r, 2);
  if ((ss + 1 == 1) && (PrevAbsHeadingCandA == 100.0)) {
    PosHH->re = 0.0;
    PosHH->im = 0.0;
    HeadingHH = 0.0;
  } else if (ss + 1 == 1) {
    HeadingHH = rt_atan2d_snf(PrevHeadingCandA_im, PrevHeadingCandA_re);
  } else {
    int iidx[20000];
    short size_tmp_idx_1;
    boolean_T b_LessThan1m1[20000];
    for (i = 0; i < 20000; i++) {
      y[i] = LessThan1m1[i];
    }
    sort(y, iidx);
    for (i = 0; i < 20000; i++) {
      b_LessThan1m1[i] = (LessThan1m1[i] == y[0]);
    }
    eml_find(b_LessThan1m1, r);
    size_tmp_idx_1 = (short)r->size[1];
    for (i = 0; i < 20000; i++) {
      b_LessThan1m1[i] = (LessThan1m1[i] == y[0]);
    }
    eml_find(b_LessThan1m1, r);
    if (r->size[1] < 1) {
      k0 = 0;
    } else {
      k0 = size_tmp_idx_1;
    }
    i = sortedCenter->size[0] * sortedCenter->size[1];
    sortedCenter->size[0] = 1;
    sortedCenter->size[1] = k0;
    emxEnsureCapacity_creal_T(sortedCenter, i);
    sortedCenter_data = sortedCenter->data;
    for (i = 0; i < k0; i++) {
      sortedCenter_data[i] = CenterCands[iidx[i] - 1];
    }
    i = sortedHeading->size[0] * sortedHeading->size[1];
    sortedHeading->size[0] = 1;
    sortedHeading->size[1] = k0;
    emxEnsureCapacity_creal_T(sortedHeading, i);
    sortedCenter_data = sortedHeading->data;
    for (i = 0; i < k0; i++) {
      sortedCenter_data[i] = HeadingCands[iidx[i] - 1];
    }
    *PosHH = mean(sortedCenter);
    HeadingCandA = mean(sortedHeading);
    HeadingHH = rt_atan2d_snf(HeadingCandA.im, HeadingCandA.re);
  }
  emxFree_int32_T(&r);
  emxFree_creal_T(&sortedHeading);
  emxFree_creal_T(&sortedCenter);
  if (HeadingHH * 0.0 == 0.0) {
    HeadingCandA.re = cos(HeadingHH);
    HeadingCandA.im = sin(HeadingHH);
  } else if (HeadingHH == 0.0) {
    HeadingCandA.re = rtNaN;
    HeadingCandA.im = 0.0;
  } else {
    HeadingCandA.re = rtNaN;
    HeadingCandA.im = rtNaN;
  }
  lastBlockLength = Xt_b_size[1] - 1;
  for (i = 0; i <= lastBlockLength; i++) {
    bsum = Xt_b_data[i].im;
    HeadingCandA_tmp = Xt_b_data[i].re;
    Xt_b_data[i].re = PosHH->re + (HeadingCandA.re * HeadingCandA_tmp -
                                   HeadingCandA.im * bsum);
    Xt_b_data[i].im = PosHH->im + (HeadingCandA.re * bsum +
                                   HeadingCandA.im * HeadingCandA_tmp);
  }
  emxInit_creal_T(&Xt_b);
  i = Xt_b->size[0] * Xt_b->size[1];
  Xt_b->size[0] = Xt_b_size[1];
  Xt_b->size[1] = Xain->size[1];
  emxEnsureCapacity_creal_T(Xt_b, i);
  sortedCenter_data = Xt_b->data;
  lastBlockLength = Xain->size[1];
  for (i = 0; i < lastBlockLength; i++) {
    k0 = Xt_b_size[1];
    for (i1 = 0; i1 < k0; i1++) {
      sortedCenter_data[i1 + Xt_b->size[0] * i].re =
          Xt_b_data[i1].re - Xain_data[i].re;
      sortedCenter_data[i1 + Xt_b->size[0] * i].im =
          Xt_b_data[i1].im - Xain_data[i].im;
    }
  }
  emxFree_creal_T(&Xain);
  emxInit_real_T(&r1, 2);
  b_abs(Xt_b, r1);
  DistErr_data = r1->data;
  emxFree_creal_T(&Xt_b);
  if ((r1->size[0] == 4) && (r1->size[1] == b_DistMap->size[1])) {
    i = TempA->size[0] * TempA->size[1];
    TempA->size[0] = 4;
    TempA->size[1] = r1->size[1];
    emxEnsureCapacity_real_T(TempA, i);
    TempA_data = TempA->data;
    lastBlockLength = r1->size[1];
    for (i = 0; i < lastBlockLength; i++) {
      for (i1 = 0; i1 < 4; i1++) {
        i2 = i1 + 4 * i;
        TempA_data[i2] = DistErr_data[i1 + r1->size[0] * i] -
                         DistMap_data[i2 + 4 * b_DistMap->size[1] * 99];
      }
    }
  } else {
    binary_expand_op_5(TempA, r1, b_DistMap);
    TempA_data = TempA->data;
  }
  emxFree_real_T(&r1);
  emxInit_boolean_T(&r2, 2);
  i = r2->size[0] * r2->size[1];
  r2->size[0] = 4;
  r2->size[1] = b_DistMap->size[1];
  emxEnsureCapacity_boolean_T(r2, i);
  x_data = r2->data;
  lastBlockLength = b_DistMap->size[1];
  for (i = 0; i < lastBlockLength; i++) {
    x_data[4 * i] = (DistMap_data[4 * i + 4 * b_DistMap->size[1] * 99] == 0.0);
    x_data[4 * i + 1] =
        (DistMap_data[(4 * i + 4 * b_DistMap->size[1] * 99) + 1] == 0.0);
    x_data[4 * i + 2] =
        (DistMap_data[(4 * i + 4 * b_DistMap->size[1] * 99) + 2] == 0.0);
    x_data[4 * i + 3] =
        (DistMap_data[(4 * i + 4 * b_DistMap->size[1] * 99) + 3] == 0.0);
  }
  k0 = (r2->size[1] << 2) - 1;
  for (lastBlockLength = 0; lastBlockLength <= k0; lastBlockLength++) {
    if (x_data[lastBlockLength]) {
      TempA_data[lastBlockLength] = 0.0;
    }
  }
  emxFree_boolean_T(&r2);
  emxInit_real_T(&TempB, 1);
  lastBlockLength = TempA->size[1] << 2;
  i = TempB->size[0];
  TempB->size[0] = lastBlockLength;
  emxEnsureCapacity_real_T(TempB, i);
  TempB_data = TempB->data;
  for (i = 0; i < lastBlockLength; i++) {
    TempB_data[i] = TempA_data[i];
  }
  i = b_x->size[0];
  b_x->size[0] = lastBlockLength;
  emxEnsureCapacity_boolean_T(b_x, i);
  x_data = b_x->data;
  for (i = 0; i < lastBlockLength; i++) {
    x_data[i] = (TempA_data[i] == 0.0);
  }
  emxFree_real_T(&TempA);
  k0 = 0;
  i = b_x->size[0];
  for (k = 0; k < i; k++) {
    k0 += x_data[k];
  }
  idx = lastBlockLength - k0;
  k0 = -1;
  for (k = 0; k < lastBlockLength; k++) {
    if ((k + 1 > b_x->size[0]) || (!x_data[k])) {
      k0++;
      TempB_data[k0] = TempB_data[k];
    }
  }
  emxFree_boolean_T(&b_x);
  i = TempB->size[0];
  if (idx < 1) {
    TempB->size[0] = 0;
  } else {
    TempB->size[0] = idx;
  }
  emxEnsureCapacity_real_T(TempB, i);
  TempB_data = TempB->data;
  nx = TempB->size[0];
  emxInit_real_T(&b_y, 1);
  i = b_y->size[0];
  b_y->size[0] = TempB->size[0];
  emxEnsureCapacity_real_T(b_y, i);
  DistErr_data = b_y->data;
  for (k = 0; k < nx; k++) {
    DistErr_data[k] = fabs(TempB_data[k]);
  }
  lastBlockLength = b_y->size[0];
  for (i = 0; i < lastBlockLength; i++) {
    bsum = DistErr_data[i];
    DistErr_data[i] = bsum * bsum;
  }
  if (b_y->size[0] == 0) {
    brm = 0.0;
  } else {
    if (b_y->size[0] <= 1024) {
      k0 = b_y->size[0];
      lastBlockLength = 0;
      nx = 1;
    } else {
      k0 = 1024;
      nx = (int)((unsigned int)b_y->size[0] >> 10);
      lastBlockLength = b_y->size[0] - (nx << 10);
      if (lastBlockLength > 0) {
        nx++;
      } else {
        lastBlockLength = 1024;
      }
    }
    brm = DistErr_data[0];
    for (k = 2; k <= k0; k++) {
      brm += DistErr_data[k - 1];
    }
    for (ss = 2; ss <= nx; ss++) {
      k0 = (ss - 1) << 10;
      bsum = DistErr_data[k0];
      if (ss == nx) {
        idx = lastBlockLength;
      } else {
        idx = 1024;
      }
      for (k = 2; k <= idx; k++) {
        bsum += DistErr_data[(k0 + k) - 1];
      }
      brm += bsum;
    }
  }
  if ((sqrt(brm / (double)b_y->size[0]) > 0.5) || (TempB->size[0] < 11)) {
    PosHH->re = 0.0;
    PosHH->im = 0.0;
  }
  emxFree_real_T(&b_y);
  emxFree_real_T(&TempB);
  return HeadingHH;
}

/* End of code generation (UWBMultiTagPos_V3_1.c) */
