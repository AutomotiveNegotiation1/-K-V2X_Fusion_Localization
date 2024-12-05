/*
 * UWBPosition_V4_1.c
 *
 * Code generation for function 'UWBPosition_V4_1'
 *
 */

/* Include files */
#include "UWBPosition_V4_1.h"
#include "PositioningSystem_V4_2_emxutil.h"
#include "PositioningSystem_V4_2_rtwutil.h"
#include "PositioningSystem_V4_2_types.h"
#include "UWBMultiTagPos_V3_1.h"
#include "UWBpos_V2_3.h"
#include "nullAssignment.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Variable Definitions */
static boolean_T s_time_prev_not_empty;

static emxArray_real_T *DistMap;

static cell_wrap_4 PosUWB2[4];

/* Function Declarations */
static void minus(double in1_data[], int in1_size[2], const double in2_data[],
                  const int in2_size[2]);

/* Function Definitions */
static void minus(double in1_data[], int in1_size[2], const double in2_data[],
                  const int in2_size[2])
{
  double b_in2_data[65];
  int i;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  if (in1_size[1] == 1) {
    loop_ub = in2_size[1];
  } else {
    loop_ub = in1_size[1];
  }
  stride_0_1 = (in2_size[1] != 1);
  stride_1_1 = (in1_size[1] != 1);
  for (i = 0; i < loop_ub; i++) {
    b_in2_data[i] = in2_data[i * stride_0_1] - in1_data[i * stride_1_1];
  }
  in1_size[0] = 1;
  in1_size[1] = loop_ub;
  if (loop_ub - 1 >= 0) {
    memcpy(&in1_data[0], &b_in2_data[0],
           (unsigned int)loop_ub * sizeof(double));
  }
}

double UWBPosition_V4_1(double s_time, double Ln, double Nanchor, double TagID,
                        const double RxIDUWB_data[], const int RxIDUWB_size[2],
                        const double RxDistOrig_data[],
                        const int RxDistOrig_size[2], const double xain_data[],
                        const int xain_size[2], const double yain_data[],
                        const int yain_size[2], const double zain_data[],
                        const double xt_b_data[], const int xt_b_size[2],
                        const double yt_b_data[], const int yt_b_size[2],
                        double zt_b, creal_T *PosUWBO, double *UWBFull)
{
  static double b_k;
  static double s_time_prev;
  emxArray_boolean_T *x;
  emxArray_int32_T *ii;
  emxArray_real_T *r;
  creal_T Xain_data[65];
  creal_T Pos3T_data[12];
  creal_T Pos3_data[12];
  creal_T TPos_data[12];
  creal_T Pos2_data[6];
  creal_T Pos1;
  double RxDist_data[65];
  double HeadingUWBO;
  double gs;
  double *DistMap_data;
  double *r1;
  int b_tmp_data[12];
  int Pos2_size[2];
  int Pos3_size[2];
  int RxDist_size[2];
  int TPos_size[2];
  int tmp_size[2];
  int x_size[2];
  int Pos3T_size_idx_1;
  int i;
  int k;
  int k0;
  int loop_ub;
  int nxout;
  int sg;
  int *ii_data;
  boolean_T idx_data[12];
  boolean_T exitg1;
  boolean_T y;
  boolean_T *c_x_data;
  DistMap_data = DistMap->data;
  if (!s_time_prev_not_empty) {
    s_time_prev = 0.0;
    s_time_prev_not_empty = true;
    b_k = 1.0;
    PosUWB2[0].f1.size[0] = 1;
    PosUWB2[0].f1.size[1] = 1;
    PosUWB2[0].f1.data[0].re = 0.0;
    PosUWB2[0].f1.data[0].im = 0.0;
    PosUWB2[1].f1.size[0] = 1;
    PosUWB2[1].f1.size[1] = 1;
    PosUWB2[1].f1.data[0].re = 0.0;
    PosUWB2[1].f1.data[0].im = 0.0;
    PosUWB2[2].f1.size[0] = 1;
    PosUWB2[2].f1.size[1] = 1;
    PosUWB2[2].f1.data[0].re = 0.0;
    PosUWB2[2].f1.data[0].im = 0.0;
    PosUWB2[3].f1.size[0] = 1;
    PosUWB2[3].f1.size[1] = 1;
    PosUWB2[3].f1.data[0].re = 0.0;
    PosUWB2[3].f1.data[0].im = 0.0;
    i = DistMap->size[0] * DistMap->size[1] * DistMap->size[2];
    DistMap->size[0] = 4;
    DistMap->size[1] = (int)Ln;
    DistMap->size[2] = 100;
    emxEnsureCapacity_real_T(DistMap, i);
    DistMap_data = DistMap->data;
    loop_ub = ((int)Ln << 2) * 100;
    for (i = 0; i < loop_ub; i++) {
      DistMap_data[i] = 0.0;
    }
  }
  gs = zain_data[0] - zt_b;
  gs *= gs;
  RxDist_size[0] = 1;
  RxDist_size[1] = RxDistOrig_size[1];
  loop_ub = RxDistOrig_size[1];
  for (k = 0; k < loop_ub; k++) {
    double varargin_1;
    varargin_1 = RxDistOrig_data[k];
    RxDist_data[k] = sqrt(varargin_1 * varargin_1 - gs);
  }
  if (s_time - s_time_prev > 0.05) {
    int tmp_data[65];
    b_k++;
    emxInit_real_T(&r, 3);
    i = r->size[0] * r->size[1] * r->size[2];
    r->size[0] = 4;
    r->size[1] = DistMap->size[1];
    r->size[2] = 99;
    emxEnsureCapacity_real_T(r, i);
    r1 = r->data;
    loop_ub = DistMap->size[1];
    for (i = 0; i < 99; i++) {
      for (nxout = 0; nxout < loop_ub; nxout++) {
        r1[4 * nxout + 4 * r->size[1] * i] =
            DistMap_data[4 * nxout + 4 * DistMap->size[1] * (i + 1)];
        r1[(4 * nxout + 4 * r->size[1] * i) + 1] =
            DistMap_data[(4 * nxout + 4 * DistMap->size[1] * (i + 1)) + 1];
        r1[(4 * nxout + 4 * r->size[1] * i) + 2] =
            DistMap_data[(4 * nxout + 4 * DistMap->size[1] * (i + 1)) + 2];
        r1[(4 * nxout + 4 * r->size[1] * i) + 3] =
            DistMap_data[(4 * nxout + 4 * DistMap->size[1] * (i + 1)) + 3];
      }
    }
    loop_ub = r->size[1];
    for (i = 0; i < 99; i++) {
      for (nxout = 0; nxout < loop_ub; nxout++) {
        DistMap_data[4 * nxout + 4 * DistMap->size[1] * i] =
            r1[4 * nxout + 4 * r->size[1] * i];
        DistMap_data[(4 * nxout + 4 * DistMap->size[1] * i) + 1] =
            r1[(4 * nxout + 4 * r->size[1] * i) + 1];
        DistMap_data[(4 * nxout + 4 * DistMap->size[1] * i) + 2] =
            r1[(4 * nxout + 4 * r->size[1] * i) + 2];
        DistMap_data[(4 * nxout + 4 * DistMap->size[1] * i) + 3] =
            r1[(4 * nxout + 4 * r->size[1] * i) + 3];
      }
    }
    emxFree_real_T(&r);
    loop_ub = DistMap->size[1];
    for (i = 0; i < loop_ub; i++) {
      DistMap_data[4 * i + 4 * DistMap->size[1] * 99] = 0.0;
      DistMap_data[(4 * i + 4 * DistMap->size[1] * 99) + 1] = 0.0;
      DistMap_data[(4 * i + 4 * DistMap->size[1] * 99) + 2] = 0.0;
      DistMap_data[(4 * i + 4 * DistMap->size[1] * 99) + 3] = 0.0;
    }
    nxout = RxIDUWB_size[1];
    loop_ub = RxIDUWB_size[1];
    for (i = 0; i < loop_ub; i++) {
      tmp_data[i] = (int)RxIDUWB_data[i] - 1;
    }
    for (i = 0; i < nxout; i++) {
      DistMap_data[(((int)TagID + 4 * tmp_data[i]) +
                    4 * DistMap->size[1] * 99) -
                   1] = RxDist_data[i];
    }
    /*  DistMap(:,1:length(xain),end) =
     * abs(transpose(PrevPosUWBH)-(xain+j*yain)); */
    /*  %  */
    /*  for ggl = 1 : length(RxDist) */
    /*      if abs(DistMap(TagID,RxIDUWB(ggl),end) - RxDist(ggl)) < 0.5 */
    /*          DistMap(TagID,RxIDUWB(ggl),end) = RxDist(ggl); */
    /*      else */
    /*          gdg = 1; */
    /*      end */
    /*  end */
    /*  [PosUWBN(k), HeadingUWBN(k), PosUWBe(k), HeadingUWBe(k)] =
     * HeadingMat(DistMap(:,:,end-2:end-1), xain, yain, xt_b, yt_b, RxIDUWB); */
  } else {
    int tmp_data[65];
    nxout = RxIDUWB_size[1];
    loop_ub = RxIDUWB_size[1];
    for (i = 0; i < loop_ub; i++) {
      tmp_data[i] = (int)RxIDUWB_data[i] - 1;
    }
    for (i = 0; i < nxout; i++) {
      DistMap_data[(((int)TagID + 4 * tmp_data[i]) +
                    4 * DistMap->size[1] * 99) -
                   1] = RxDist_data[i];
    }
  }
  if (RxIDUWB_size[1] > 1) {
    if (Nanchor != 0.0) {
      UWBpos_V2_3(Nanchor, RxIDUWB_data, RxDist_data, xain_data, yain_data,
                  &Pos1, Pos2_data, Pos2_size, Pos3_data, Pos3_size);
    } else {
      Pos3_size[1] = 1;
      Pos3_data[0].re = 0.0;
      Pos3_data[0].im = 0.0;
    }
  } else {
    Pos3_size[1] = 1;
    Pos3_data[0].re = 0.0;
    Pos3_data[0].im = 0.0;
  }
  TPos_size[0] = 1;
  TPos_size[1] = Pos3_size[1];
  loop_ub = Pos3_size[1];
  if (loop_ub - 1 >= 0) {
    memset(&TPos_data[0], 0, (unsigned int)loop_ub * sizeof(creal_T));
  }
  if (xain_size[1] == yain_size[1]) {
    loop_ub = xain_size[1];
    for (i = 0; i < loop_ub; i++) {
      gs = yain_data[i];
      Xain_data[i].re = xain_data[i] + 0.0 * gs;
      Xain_data[i].im = gs;
    }
  } else {
    binary_expand_op_1(Xain_data, Pos2_size, xain_data, xain_size, yain_data,
                       yain_size);
  }
  sg = 1;
  gs = 1.0;
  Pos3T_size_idx_1 = Pos3_size[1];
  loop_ub = Pos3_size[1];
  if (loop_ub - 1 >= 0) {
    memset(&Pos3T_data[0], 0, (unsigned int)loop_ub * sizeof(creal_T));
  }
  i = (int)((double)Pos3_size[1] / 2.0);
  for (k0 = 0; k0 < i; k0++) {
    nxout = k0 << 1;
    Pos1 = Pos3_data[nxout + 1];
    if ((Pos3_data[nxout].re != Pos1.re) ||
        (Pos3_data[nxout].im != Pos3_data[nxout + 1].im)) {
      Pos3T_data[(int)gs - 1] = Pos3_data[nxout];
      Pos3T_data[(int)gs] = Pos1;
      gs += 2.0;
    }
  }
  nxout = 0;
  for (k = 0; k < Pos3T_size_idx_1; k++) {
    y = ((Pos3T_data[k].re == 0.0) && (Pos3T_data[k].im == 0.0));
    idx_data[k] = y;
    nxout += y;
  }
  nxout = Pos3_size[1] - nxout;
  k0 = -1;
  for (k = 0; k < Pos3T_size_idx_1; k++) {
    if ((k + 1 > Pos3T_size_idx_1) || (!idx_data[k])) {
      k0++;
      Pos3T_data[k0] = Pos3T_data[k];
    }
  }
  if (nxout < 1) {
    Pos3T_size_idx_1 = 0;
  } else {
    Pos3T_size_idx_1 = nxout;
  }
  for (k0 = 0; k0 < Pos3T_size_idx_1; k0++) {
    double x_data[65];
    boolean_T b_x_data[65];
    nxout = RxIDUWB_size[1];
    x_size[0] = 1;
    x_size[1] = RxIDUWB_size[1];
    for (k = 0; k < nxout; k++) {
      i = (int)RxIDUWB_data[k] - 1;
      x_data[k] = rt_hypotd_snf(Pos3T_data[k0].re - Xain_data[i].re,
                                Pos3T_data[k0].im - Xain_data[i].im);
    }
    if (RxDist_size[1] == RxIDUWB_size[1]) {
      loop_ub = RxDist_size[1] - 1;
      x_size[1] = RxDist_size[1];
      for (i = 0; i <= loop_ub; i++) {
        x_data[i] = RxDist_data[i] - x_data[i];
      }
    } else {
      minus(x_data, x_size, RxDist_data, RxDist_size);
    }
    nxout = x_size[1];
    for (k = 0; k < nxout; k++) {
      b_x_data[k] = (fabs(x_data[k]) < 5.0);
    }
    y = (x_size[1] != 0);
    if (y) {
      k = 0;
      exitg1 = false;
      while ((!exitg1) && (k <= nxout - 1)) {
        if (!b_x_data[k]) {
          y = false;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (y) {
      TPos_data[sg - 1] = Pos3T_data[k0];
      sg++;
    }
  }
  tmp_size[0] = 1;
  loop_ub = Pos3_size[1] - sg;
  tmp_size[1] = loop_ub + 1;
  for (i = 0; i <= loop_ub; i++) {
    b_tmp_data[i] = sg + i;
  }
  nullAssignment(TPos_data, TPos_size, b_tmp_data, tmp_size);
  if (TPos_size[1] == 0) {
    PosUWB2[(int)TagID - 1].f1.size[0] = 1;
    PosUWB2[(int)TagID - 1].f1.size[1] = Pos3_size[1];
    loop_ub = Pos3_size[1];
    for (i = 0; i < loop_ub; i++) {
      PosUWB2[(int)TagID - 1].f1.data[i] = Pos3_data[i];
    }
  } else {
    /*  PosUWB2{TagID} = Pos3; */
    PosUWB2[(int)TagID - 1].f1.size[0] = 1;
    PosUWB2[(int)TagID - 1].f1.size[1] = TPos_size[1];
    loop_ub = TPos_size[1];
    for (i = 0; i < loop_ub; i++) {
      PosUWB2[(int)TagID - 1].f1.data[i] = TPos_data[i];
    }
  }
  /*  DistFromPrev = abs(PrevPosH(TagID)-Pos2); */
  /*  DistFromPrevUWB = abs(PrevPosUWBH(TagID)-Pos2); */
  /*   */
  /*  if (PrevPosUWBH(TagID)~=0) && (abs(PrevPosH(TagID)-PrevPosUWBH(TagID))<2)
   */
  /*      [DistFromPrevSorted, sortedIndx] = sort(DistFromPrev+DistFromPrevUWB);
   */
  /*      PosUWB2{TagID} = Pos2(sortedIndx(1:min(length(sortedIndx)))); */
  /*  else */
  /*      PosUWB2{TagID} = Pos2; */
  /*  end */
  gs = 0.0;
  loop_ub = DistMap->size[1];
  emxInit_boolean_T(&x, 2);
  emxInit_int32_T(&ii, 2);
  for (Pos3T_size_idx_1 = 0; Pos3T_size_idx_1 < 4; Pos3T_size_idx_1++) {
    i = x->size[0] * x->size[1];
    x->size[0] = 1;
    x->size[1] = DistMap->size[1];
    emxEnsureCapacity_boolean_T(x, i);
    c_x_data = x->data;
    for (i = 0; i < loop_ub; i++) {
      c_x_data[i] = (DistMap_data[(Pos3T_size_idx_1 + 4 * i) +
                                  4 * DistMap->size[1] * 99] != 0.0);
    }
    k0 = x->size[1];
    sg = 0;
    i = ii->size[0] * ii->size[1];
    ii->size[0] = 1;
    ii->size[1] = x->size[1];
    emxEnsureCapacity_int32_T(ii, i);
    ii_data = ii->data;
    nxout = 0;
    exitg1 = false;
    while ((!exitg1) && (nxout <= k0 - 1)) {
      if (c_x_data[nxout]) {
        sg++;
        ii_data[sg - 1] = nxout + 1;
        if (sg >= k0) {
          exitg1 = true;
        } else {
          nxout++;
        }
      } else {
        nxout++;
      }
    }
    if (x->size[1] == 1) {
      if (sg == 0) {
        ii->size[1] = 0;
      }
    } else {
      i = ii->size[0] * ii->size[1];
      if (sg < 1) {
        ii->size[1] = 0;
      } else {
        ii->size[1] = sg;
      }
      emxEnsureCapacity_int32_T(ii, i);
    }
    if (ii->size[1] != 0) {
      gs++;
    }
  }
  emxFree_int32_T(&ii);
  emxFree_boolean_T(&x);
  if (gs > 2.0) {
    /*  if length(DistMap(:,:,end-1)~=0)==16 */
    /*  [PosUWBN(k), HeadingUWBN(k), PosUWBe(k), HeadingUWBe(k)] =
     * HeadingMat_2(DistMap(:,:,end-5:end-1), xain, yain, xt_b, yt_b, RxIDUWB);
     */
    /* %%% 테스트를 위해 잠시 off */
    HeadingUWBO = UWBMultiTagPos_V3_1(
        PosUWB2, xt_b_data, xt_b_size, yt_b_data, yt_b_size, xain_data,
        xain_size, yain_data, yain_size, DistMap, Ln, PosUWBO);
    /*  PosUWBO = (PosUWBO+PosUWBN(k))/2; */
    /*  HeadingUWBO = HeadingUWBN(k); */
    /*  [PosUWBO, HeadingUWBO] = UWBMultiTagPos_V3(PosUWB2, xt_b, yt_b, xain,
     * yain, DistMap); */
    /*  [PosUWBO, HeadingUWBO] = UWBMultiTagPos_V2(PosUWB2, xt_b, yt_b); */
    *UWBFull = 1.0;
  } else {
    /*  [PosUWBO, HeadingUWBO] = UWBMultiTagPos_V2(PosUWB2, xt_b, yt_b); */
    PosUWBO->re = 0.0;
    PosUWBO->im = 0.0;
    HeadingUWBO = 0.0;
    *UWBFull = 0.0;
  }
  s_time_prev = s_time;
  return HeadingUWBO;
}

void UWBPosition_V4_1_free(void)
{
  emxFree_real_T(&DistMap);
}

void UWBPosition_V4_1_init(void)
{
  emxInitMatrix_cell_wrap_4(PosUWB2);
  emxInit_real_T(&DistMap, 3);
  s_time_prev_not_empty = false;
}

void binary_expand_op_1(creal_T in1_data[], int in1_size[2],
                        const double in2_data[], const int in2_size[2],
                        const double in4_data[], const int in4_size[2])
{
  int i;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1_size[0] = 1;
  if (in4_size[1] == 1) {
    loop_ub = in2_size[1];
  } else {
    loop_ub = in4_size[1];
  }
  in1_size[1] = loop_ub;
  stride_0_1 = (in2_size[1] != 1);
  stride_1_1 = (in4_size[1] != 1);
  for (i = 0; i < loop_ub; i++) {
    double d;
    d = in4_data[i * stride_1_1];
    in1_data[i].re = in2_data[i * stride_0_1] + 0.0 * d;
    in1_data[i].im = d;
  }
}

/* End of code generation (UWBPosition_V4_1.c) */
