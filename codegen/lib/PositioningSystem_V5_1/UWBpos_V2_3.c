/*
 * UWBpos_V2_3.c
 *
 * Code generation for function 'UWBpos_V2_3'
 *
 */

/* Include files */
#include "UWBpos_V2_3.h"
#include "PositioningSystem_V5_1_emxutil.h"
#include "PositioningSystem_V5_1_rtwutil.h"
#include "PositioningSystem_V5_1_types.h"
#include "inv.h"
#include "mldivide.h"
#include "nullAssignment.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
void UWBpos_V2_3(double Nanchor, const double RxIDin_data[],
                 const double RxDistin_data[], const double xain_data[],
                 const double yain_data[], creal_T *Pos1, creal_T Pos2_data[],
                 int Pos2_size[2], creal_T Pos3_data[], int Pos3_size[2])
{
  emxArray_creal_T *x;
  emxArray_real_T *b_y;
  creal_T tmp_data[12];
  creal_T *x_data;
  double A_data[128];
  double xa_data[65];
  double ya_data[65];
  double X1[2];
  double d;
  double *b_y_data;
  int A_size[2];
  int b_A_size[2];
  int b_loop_ub;
  int coffset;
  int i;
  int k;
  int loop_ub;
  int v;
  boolean_T b_x[2];
  if (Nanchor < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = (int)Nanchor;
  }
  for (i = 0; i < loop_ub; i++) {
    d = RxIDin_data[i];
    xa_data[i] = xain_data[(int)d - 1];
    ya_data[i] = yain_data[(int)d - 1];
  }
  if (Nanchor < 1.0) {
    b_loop_ub = -1;
  } else {
    b_loop_ub = (int)Nanchor - 1;
  }
  memset(&Pos2_data[0], 0, 6U * sizeof(creal_T));
  Pos3_size[0] = 1;
  Pos3_size[1] = 12;
  memset(&Pos3_data[0], 0, 12U * sizeof(creal_T));
  v = 1;
  if (Nanchor == 2.0) {
    double Pos[4];
    double AA;
    double a_tmp;
    double b_a_tmp;
    double bkj;
    double c_a_tmp;
    double d_a_tmp;
    int b_tmp_data[4];
    int boffset;
    /*  Xa = [0 1]; */
    /*  Ya = [0 0]; */
    /*  dist = [1 1]; */
    boffset = (int)RxIDin_data[0] - 1;
    coffset = (int)RxIDin_data[1] - 1;
    a_tmp = xain_data[coffset];
    b_a_tmp = xain_data[boffset];
    bkj = b_a_tmp - a_tmp;
    c_a_tmp = yain_data[coffset];
    d_a_tmp = yain_data[boffset];
    AA = d_a_tmp - c_a_tmp;
    AA = sqrt(bkj * bkj + AA * AA);
    /*  AA = 2;B=1;C=0.9 */
    bkj = ((AA + RxDistin_data[0]) + RxDistin_data[1]) / 2.0;
    d = bkj * (bkj - AA) * (bkj - RxDistin_data[0]) * (bkj - RxDistin_data[1]);
    if (d > 0.0) {
      double dv[4];
      double dv1[4];
      double y_tmp[4];
      double Pos_idx_0;
      double Pos_idx_0_tmp;
      double Pos_tmp;
      double b_Pos_idx_0_tmp;
      double b_Pos_tmp;
      double c_Pos_idx_0_tmp;
      double d1;
      double d2;
      double d3;
      AA = 2.0 * sqrt(d) / AA;
      /*  if (B^2-((B^2-C^2+AA^2)/(2*AA))^2) > 0 */
      /*      d = sqrt(B^2-((B^2-C^2+AA^2)/(2*AA))^2); */
      /*  else */
      /*      d = 0; */
      /*  end */
      Pos_tmp = c_a_tmp - d_a_tmp;
      b_Pos_tmp = a_tmp - b_a_tmp;
      Pos[1] = 2.0 * b_Pos_tmp;
      Pos[3] = 2.0 * Pos_tmp;
      /*  Y1 =
       * [d*sqrt((Ya(2)-Ya(1))^2+(Xa(2)-Xa(1))^2)+Xa(1)*Ya(2)-Xa(2)*Ya(1);(C^2-B^2-Xa(1)^2+Xa(2)^2-Ya(1)^2+Ya(2)^2)];
       */
      /*  Y2 =
       * [-d*sqrt((Ya(2)-Ya(1))^2+(Xa(2)-Xa(1))^2)+Xa(1)*Ya(2)-Xa(2)*Ya(1);(C^2-B^2-Xa(1)^2+Xa(2)^2-Ya(1)^2+Ya(2)^2)];
       */
      y_tmp[0] = Pos_tmp;
      y_tmp[1] = -b_Pos_tmp;
      y_tmp[2] = Pos[1];
      y_tmp[3] = Pos[3];
      d = Pos[1];
      d1 = Pos[3];
      for (i = 0; i < 2; i++) {
        d2 = y_tmp[i + 2];
        d3 = y_tmp[i];
        Pos[i] = d3 * Pos_tmp + d2 * d;
        Pos[i + 2] = d3 * -b_Pos_tmp + d2 * d1;
      }
      /*  if X1 == X2 */
      /*      Pos = [0;0]; */
      /*  else */
      inv(Pos, dv);
      inv(Pos, dv1);
      Pos_idx_0_tmp = sqrt(Pos_tmp * Pos_tmp + b_Pos_tmp * b_Pos_tmp);
      b_Pos_idx_0_tmp = b_a_tmp * c_a_tmp;
      c_Pos_idx_0_tmp = d_a_tmp * a_tmp;
      Pos_idx_0 = (AA * Pos_idx_0_tmp + b_Pos_idx_0_tmp) - c_Pos_idx_0_tmp;
      bkj = ((((-(RxDistin_data[1] * RxDistin_data[1]) +
                RxDistin_data[0] * RxDistin_data[0]) -
               b_a_tmp * b_a_tmp) +
              a_tmp * a_tmp) -
             d_a_tmp * d_a_tmp) +
            c_a_tmp * c_a_tmp;
      d = y_tmp[2];
      d1 = y_tmp[3];
      for (i = 0; i < 2; i++) {
        d2 = dv[i + 2];
        d3 = dv[i];
        X1[i] = (d3 * Pos_tmp + d2 * -b_Pos_tmp) * Pos_idx_0 +
                (d3 * d + d2 * d1) * bkj;
        d2 = dv1[i + 2];
        d3 = dv1[i];
        dv[i] = d3 * Pos_tmp + d2 * -b_Pos_tmp;
        dv[i + 2] = d3 * d + d2 * d1;
      }
      Pos_idx_0 = (-AA * Pos_idx_0_tmp + b_Pos_idx_0_tmp) - c_Pos_idx_0_tmp;
      Pos[0] = X1[0];
      Pos[1] = dv[0] * Pos_idx_0 + dv[2] * bkj;
      Pos[2] = X1[1];
      Pos[3] = dv[1] * Pos_idx_0 + dv[3] * bkj;
    } else {
      if (loop_ub == 0) {
        b_a_tmp = 0.0;
      } else {
        for (k = 2; k <= loop_ub; k++) {
          b_a_tmp += xa_data[k - 1];
        }
      }
      if (loop_ub == 0) {
        d_a_tmp = 0.0;
      } else {
        for (k = 2; k <= loop_ub; k++) {
          d_a_tmp += ya_data[k - 1];
        }
      }
      X1[0] = b_a_tmp / (double)loop_ub;
      X1[1] = d_a_tmp / (double)loop_ub;
      Pos[0] = X1[0];
      Pos[1] = X1[0];
      Pos[2] = X1[1];
      Pos[3] = X1[1];
    }
    /*  end */
    Pos1->re = 0.0;
    Pos1->im = 0.0;
    Pos2_data[0].re = Pos[0];
    Pos2_data[0].im = Pos[2];
    Pos2_data[1].re = Pos[1];
    Pos2_data[1].im = Pos[3];
    A_size[0] = 1;
    A_size[1] = 6;
    memcpy(&tmp_data[0], &Pos2_data[0], 6U * sizeof(creal_T));
    b_A_size[0] = 1;
    b_A_size[1] = 4;
    b_tmp_data[0] = 3;
    b_tmp_data[1] = 4;
    b_tmp_data[2] = 5;
    b_tmp_data[3] = 6;
    nullAssignment(tmp_data, A_size, b_tmp_data, b_A_size);
    Pos2_size[0] = 1;
    Pos2_size[1] = A_size[1];
    loop_ub = A_size[1];
    Pos3_size[0] = 1;
    Pos3_size[1] = A_size[1];
    for (i = 0; i < loop_ub; i++) {
      creal_T dc;
      dc = tmp_data[i];
      Pos2_data[i] = dc;
      Pos3_data[i] = dc;
    }
  } else if (Nanchor >= 3.0) {
    double b_x_data[144];
    double b_A_data[128];
    double y_data[64];
    double meanTempW_data[12];
    double Pos[4];
    double AA;
    double Pos_idx_0;
    double bkj;
    double d1;
    double d2;
    double d3;
    int boffset;
    boffset = b_loop_ub << 1;
    if (boffset - 1 >= 0) {
      memset(&A_data[0], 0, (unsigned int)boffset * sizeof(double));
    }
    /*  if (ya(2)-y(1))*(xa(3)-xa(1)) ~= (ya(3)-y(1))*(xa(2)-xa(1)) */
    for (k = 0; k < b_loop_ub; k++) {
      y_data[k] = 0.0;
      d = RxDistin_data[k];
      if ((d != 0.0) && (RxDistin_data[b_loop_ub] != 0.0)) {
        d1 = xa_data[k];
        d2 = xa_data[b_loop_ub];
        A_data[k] = -2.0 * (d1 - d2);
        d3 = ya_data[k];
        bkj = ya_data[b_loop_ub];
        A_data[k + b_loop_ub] = -2.0 * (d3 - bkj);
        AA = RxDistin_data[b_loop_ub];
        y_data[k] =
            ((d * d - AA * AA) - (d1 * d1 - d2 * d2)) - (d3 * d3 - bkj * bkj);
      }
    }
    /*  else */
    /*       */
    /*  end */
    /*  Pos = (A'*A)\(A'*y); */
    for (loop_ub = 0; loop_ub < 2; loop_ub++) {
      coffset = loop_ub << 1;
      boffset = loop_ub * b_loop_ub;
      Pos[coffset] = 0.0;
      Pos[coffset + 1] = 0.0;
      for (k = 0; k < b_loop_ub; k++) {
        bkj = A_data[boffset + k];
        Pos[coffset] += A_data[k] * bkj;
        Pos[coffset + 1] += A_data[b_loop_ub + k] * bkj;
      }
    }
    A_size[0] = 2;
    A_size[1] = b_loop_ub;
    for (i = 0; i < b_loop_ub; i++) {
      b_A_data[2 * i] = A_data[i];
      b_A_data[2 * i + 1] = A_data[i + b_loop_ub];
    }
    mldivide(Pos, b_A_data, A_size, A_data, b_A_size);
    i = b_A_size[1];
    Pos_idx_0 = 0.0;
    bkj = 0.0;
    for (k = 0; k < i; k++) {
      boffset = k << 1;
      d = y_data[k];
      Pos_idx_0 += A_data[boffset] * d;
      bkj += A_data[boffset + 1] * d;
    }
    if (rtIsNaN(Pos_idx_0) || rtIsNaN(bkj)) {
      Pos_idx_0 = 0.0;
      bkj = 0.0;
    }
    /*  Prob = sqrt(mean(abs((xa - Pos(1)).^2 + (ya - Pos(2)).^2 - dist.^2)));
     */
    Pos1->re = Pos_idx_0;
    Pos1->im = bkj;
    i = (int)(Nanchor - 1.0);
    for (k = 0; k < i; k++) {
      loop_ub = (int)(Nanchor + (1.0 - (((double)k + 1.0) + 1.0)));
      if (loop_ub - 1 >= 0) {
        b_x[0] = (RxDistin_data[k] != 0.0);
      }
      for (b_loop_ub = 0; b_loop_ub < loop_ub; b_loop_ub++) {
        boolean_T exitg1;
        boolean_T y;
        boffset = (k + b_loop_ub) + 1;
        b_x[1] = (RxDistin_data[boffset] != 0.0);
        y = true;
        coffset = 0;
        exitg1 = false;
        while ((!exitg1) && (coffset < 2)) {
          if (!b_x[coffset]) {
            y = false;
            exitg1 = true;
          } else {
            coffset++;
          }
        }
        if (y) {
          /*  Xa = [0 1]; */
          /*  Ya = [0 0]; */
          /*  dist = [1 1]; */
          bkj = xa_data[k] - xa_data[boffset];
          AA = ya_data[k] - ya_data[boffset];
          AA = sqrt(bkj * bkj + AA * AA);
          /*  AA = 2;B=1;C=0.9 */
          bkj = ((AA + RxDistin_data[k]) + RxDistin_data[boffset]) / 2.0;
          d = bkj * (bkj - AA) * (bkj - RxDistin_data[k]) *
              (bkj - RxDistin_data[boffset]);
          if (d > 0.0) {
            double dv[4];
            double dv1[4];
            double y_tmp[4];
            double Pos_idx_0_tmp;
            double Pos_tmp;
            double b_Pos_idx_0_tmp;
            double b_Pos_tmp;
            double c_Pos_idx_0_tmp;
            AA = 2.0 * sqrt(d) / AA;
            /*  if (B^2-((B^2-C^2+AA^2)/(2*AA))^2) > 0 */
            /*      d = sqrt(B^2-((B^2-C^2+AA^2)/(2*AA))^2); */
            /*  else */
            /*      d = 0; */
            /*  end */
            Pos_tmp = ya_data[boffset] - ya_data[k];
            b_Pos_tmp = xa_data[boffset] - xa_data[k];
            Pos[1] = 2.0 * b_Pos_tmp;
            Pos[3] = 2.0 * Pos_tmp;
            /*  Y1 =
             * [d*sqrt((Ya(2)-Ya(1))^2+(Xa(2)-Xa(1))^2)+Xa(1)*Ya(2)-Xa(2)*Ya(1);(C^2-B^2-Xa(1)^2+Xa(2)^2-Ya(1)^2+Ya(2)^2)];
             */
            /*  Y2 =
             * [-d*sqrt((Ya(2)-Ya(1))^2+(Xa(2)-Xa(1))^2)+Xa(1)*Ya(2)-Xa(2)*Ya(1);(C^2-B^2-Xa(1)^2+Xa(2)^2-Ya(1)^2+Ya(2)^2)];
             */
            y_tmp[0] = Pos_tmp;
            y_tmp[1] = -b_Pos_tmp;
            y_tmp[2] = Pos[1];
            y_tmp[3] = Pos[3];
            d = Pos[1];
            d1 = Pos[3];
            for (coffset = 0; coffset < 2; coffset++) {
              d2 = y_tmp[coffset + 2];
              d3 = y_tmp[coffset];
              Pos[coffset] = d3 * Pos_tmp + d2 * d;
              Pos[coffset + 2] = d3 * -b_Pos_tmp + d2 * d1;
            }
            /*  if X1 == X2 */
            /*      Pos = [0;0]; */
            /*  else */
            inv(Pos, dv);
            inv(Pos, dv1);
            Pos_idx_0_tmp = sqrt(Pos_tmp * Pos_tmp + b_Pos_tmp * b_Pos_tmp);
            b_Pos_idx_0_tmp = xa_data[k] * ya_data[boffset];
            c_Pos_idx_0_tmp = ya_data[k] * xa_data[boffset];
            Pos_idx_0 =
                (AA * Pos_idx_0_tmp + b_Pos_idx_0_tmp) - c_Pos_idx_0_tmp;
            bkj = ((((-(RxDistin_data[boffset] * RxDistin_data[boffset]) +
                      RxDistin_data[k] * RxDistin_data[k]) -
                     xa_data[k] * xa_data[k]) +
                    xa_data[boffset] * xa_data[boffset]) -
                   ya_data[k] * ya_data[k]) +
                  ya_data[boffset] * ya_data[boffset];
            d = y_tmp[2];
            d1 = y_tmp[3];
            for (coffset = 0; coffset < 2; coffset++) {
              d2 = dv[coffset + 2];
              d3 = dv[coffset];
              X1[coffset] = (d3 * Pos_tmp + d2 * -b_Pos_tmp) * Pos_idx_0 +
                            (d3 * d + d2 * d1) * bkj;
              d2 = dv1[coffset + 2];
              d3 = dv1[coffset];
              dv[coffset] = d3 * Pos_tmp + d2 * -b_Pos_tmp;
              dv[coffset + 2] = d3 * d + d2 * d1;
            }
            Pos_idx_0 =
                (-AA * Pos_idx_0_tmp + b_Pos_idx_0_tmp) - c_Pos_idx_0_tmp;
            Pos[0] = X1[0];
            Pos[1] = dv[0] * Pos_idx_0 + dv[2] * bkj;
            Pos[2] = X1[1];
            Pos[3] = dv[1] * Pos_idx_0 + dv[3] * bkj;
          } else {
            X1[0] = (xa_data[k] + xa_data[boffset]) / 2.0;
            X1[1] = (ya_data[k] + ya_data[boffset]) / 2.0;
            Pos[0] = X1[0];
            Pos[1] = X1[0];
            Pos[2] = X1[1];
            Pos[3] = X1[1];
          }
          /*  end */
          coffset = (v << 1) - 1;
          Pos3_data[coffset - 1].re = Pos[0];
          Pos3_data[coffset - 1].im = Pos[2];
          Pos3_data[coffset].re = Pos[1];
          Pos3_data[coffset].im = Pos[3];
          v++;
        }
      }
    }
    emxInit_creal_T(&x);
    i = x->size[0] * x->size[1];
    x->size[0] = 12;
    x->size[1] = 12;
    emxEnsureCapacity_creal_T(x, i);
    x_data = x->data;
    for (i = 0; i < 12; i++) {
      for (loop_ub = 0; loop_ub < 12; loop_ub++) {
        x_data[loop_ub + x->size[0] * i].re =
            Pos3_data[i].re - Pos3_data[loop_ub].re;
        x_data[loop_ub + x->size[0] * i].im =
            Pos3_data[i].im - Pos3_data[loop_ub].im;
      }
    }
    emxInit_real_T(&b_y, 2);
    i = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 12;
    b_y->size[1] = 12;
    emxEnsureCapacity_real_T(b_y, i);
    b_y_data = b_y->data;
    for (k = 0; k < 144; k++) {
      b_y_data[k] = rt_hypotd_snf(x_data[k].re, x_data[k].im);
    }
    emxFree_creal_T(&x);
    for (i = 0; i < 144; i++) {
      bkj = b_y_data[i];
      b_x_data[i] = bkj * bkj;
    }
    emxFree_real_T(&b_y);
    for (coffset = 0; coffset < 12; coffset++) {
      boffset = coffset * 12;
      d = b_x_data[boffset];
      for (k = 0; k < 11; k++) {
        d += b_x_data[(boffset + k) + 1];
      }
      meanTempW_data[coffset] = d / 12.0;
    }
    Pos2_size[0] = 1;
    Pos2_size[1] = 6;
    for (coffset = 0; coffset < 6; coffset++) {
      i = coffset << 1;
      d = meanTempW_data[i];
      d1 = meanTempW_data[i + 1];
      if ((d > d1) || (rtIsNaN(d) && (!rtIsNaN(d1)))) {
        boffset = 1;
      } else {
        boffset = 0;
      }
      /*  meanTemp(kk+1) = v; */
      Pos2_data[coffset] = Pos3_data[i + boffset];
    }
  } else {
    Pos1->re = 0.0;
    Pos1->im = 0.0;
    Pos2_size[0] = 1;
    Pos2_size[1] = 1;
    Pos2_data[0].re = 0.0;
    Pos2_data[0].im = 0.0;
  }
}

/* End of code generation (UWBpos_V2_3.c) */
