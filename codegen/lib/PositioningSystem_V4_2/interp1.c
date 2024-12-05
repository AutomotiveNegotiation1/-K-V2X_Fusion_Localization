/*
 * interp1.c
 *
 * Code generation for function 'interp1'
 *
 */

/* Include files */
#include "interp1.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
double interp1(const double varargin_1_data[], const int varargin_1_size[2],
               const double varargin_2_data[], const int varargin_2_size[2],
               double varargin_3)
{
  double pp_breaks_data[100];
  double y_data[100];
  double Vq;
  int j2;
  int low_i;
  int mid_i;
  int nx_tmp;
  int pp_breaks_size_idx_1;
  mid_i = varargin_2_size[1];
  j2 = varargin_2_size[1];
  if (j2 - 1 >= 0) {
    memcpy(&y_data[0], &varargin_2_data[0], (unsigned int)j2 * sizeof(double));
  }
  pp_breaks_size_idx_1 = varargin_1_size[1];
  j2 = varargin_1_size[1];
  if (j2 - 1 >= 0) {
    memcpy(&pp_breaks_data[0], &varargin_1_data[0],
           (unsigned int)j2 * sizeof(double));
  }
  nx_tmp = varargin_1_size[1] - 1;
  low_i = 0;
  int exitg1;
  do {
    exitg1 = 0;
    if (low_i <= nx_tmp) {
      if (rtIsNaN(varargin_1_data[low_i])) {
        exitg1 = 1;
      } else {
        low_i++;
      }
    } else {
      double pp_coefs_data[396];
      double xtmp;
      int i;
      int yoffset;
      boolean_T has_endslopes;
      if (varargin_1_data[1] < varargin_1_data[0]) {
        i = (nx_tmp + 1) >> 1;
        for (low_i = 0; low_i < i; low_i++) {
          xtmp = pp_breaks_data[low_i];
          j2 = nx_tmp - low_i;
          pp_breaks_data[low_i] = pp_breaks_data[j2];
          pp_breaks_data[j2] = xtmp;
        }
        i = varargin_2_size[1] >> 1;
        for (low_i = 0; low_i < i; low_i++) {
          j2 = (varargin_2_size[1] - low_i) - 1;
          xtmp = y_data[low_i];
          y_data[low_i] = y_data[j2];
          y_data[j2] = xtmp;
        }
      }
      has_endslopes = (mid_i == pp_breaks_size_idx_1 + 2);
      if ((pp_breaks_size_idx_1 <= 3) && (!has_endslopes)) {
        double r;
        nx_tmp = 3;
        xtmp = pp_breaks_data[1] - pp_breaks_data[0];
        r = (y_data[1] - y_data[0]) / xtmp;
        pp_coefs_data[0] =
            ((y_data[2] - y_data[1]) / (pp_breaks_data[2] - pp_breaks_data[1]) -
             r) /
            (pp_breaks_data[2] - pp_breaks_data[0]);
        pp_coefs_data[1] = r - pp_coefs_data[0] * xtmp;
        pp_coefs_data[2] = y_data[0];
        xtmp = pp_breaks_data[0];
        r = pp_breaks_data[2];
        pp_breaks_size_idx_1 = 2;
        pp_breaks_data[0] = xtmp;
        pp_breaks_data[1] = r;
      } else {
        double md_data[100];
        double s_data[100];
        double dvdf_data[99];
        double dx_data[99];
        double d31;
        double dnnm2;
        double r;
        signed char szs_idx_1;
        if (has_endslopes) {
          szs_idx_1 = (signed char)(mid_i - 2);
          yoffset = 1;
        } else {
          szs_idx_1 = (signed char)mid_i;
          yoffset = 0;
        }
        i = (unsigned char)(pp_breaks_size_idx_1 - 1);
        for (low_i = 0; low_i < i; low_i++) {
          xtmp = pp_breaks_data[low_i + 1] - pp_breaks_data[low_i];
          dx_data[low_i] = xtmp;
          j2 = yoffset + low_i;
          dvdf_data[low_i] = (y_data[j2 + 1] - y_data[j2]) / xtmp;
        }
        for (low_i = 2; low_i <= nx_tmp; low_i++) {
          s_data[low_i - 1] = 3.0 * (dx_data[low_i - 1] * dvdf_data[low_i - 2] +
                                     dx_data[low_i - 2] * dvdf_data[low_i - 1]);
        }
        if (has_endslopes) {
          d31 = 0.0;
          dnnm2 = 0.0;
          s_data[0] = y_data[0] * dx_data[1];
          s_data[pp_breaks_size_idx_1 - 1] = dx_data[pp_breaks_size_idx_1 - 3] *
                                             y_data[pp_breaks_size_idx_1 + 1];
        } else {
          d31 = pp_breaks_data[2] - pp_breaks_data[0];
          dnnm2 = pp_breaks_data[pp_breaks_size_idx_1 - 1] -
                  pp_breaks_data[pp_breaks_size_idx_1 - 3];
          s_data[0] = ((dx_data[0] + 2.0 * d31) * dx_data[1] * dvdf_data[0] +
                       dx_data[0] * dx_data[0] * dvdf_data[1]) /
                      d31;
          xtmp = dx_data[pp_breaks_size_idx_1 - 2];
          s_data[pp_breaks_size_idx_1 - 1] =
              ((xtmp + 2.0 * dnnm2) * dx_data[pp_breaks_size_idx_1 - 3] *
                   dvdf_data[pp_breaks_size_idx_1 - 2] +
               xtmp * xtmp * dvdf_data[pp_breaks_size_idx_1 - 3]) /
              dnnm2;
        }
        md_data[0] = dx_data[1];
        xtmp = dx_data[pp_breaks_size_idx_1 - 3];
        md_data[pp_breaks_size_idx_1 - 1] = xtmp;
        for (low_i = 2; low_i <= nx_tmp; low_i++) {
          md_data[low_i - 1] = 2.0 * (dx_data[low_i - 1] + dx_data[low_i - 2]);
        }
        r = dx_data[1] / md_data[0];
        md_data[1] -= r * d31;
        s_data[1] -= r * s_data[0];
        for (low_i = 3; low_i <= nx_tmp; low_i++) {
          r = dx_data[low_i - 1] / md_data[low_i - 2];
          md_data[low_i - 1] -= r * dx_data[low_i - 3];
          s_data[low_i - 1] -= r * s_data[low_i - 2];
        }
        r = dnnm2 / md_data[pp_breaks_size_idx_1 - 2];
        md_data[pp_breaks_size_idx_1 - 1] -= r * xtmp;
        s_data[pp_breaks_size_idx_1 - 1] -=
            r * s_data[pp_breaks_size_idx_1 - 2];
        s_data[pp_breaks_size_idx_1 - 1] /= md_data[pp_breaks_size_idx_1 - 1];
        for (low_i = nx_tmp; low_i >= 2; low_i--) {
          s_data[low_i - 1] =
              (s_data[low_i - 1] - dx_data[low_i - 2] * s_data[low_i]) /
              md_data[low_i - 1];
        }
        s_data[0] = (s_data[0] - d31 * s_data[1]) / md_data[0];
        nx_tmp = 4;
        for (j2 = 0; j2 < i; j2++) {
          xtmp = dvdf_data[j2];
          r = s_data[j2];
          d31 = dx_data[j2];
          dnnm2 = (xtmp - r) / d31;
          xtmp = (s_data[j2 + 1] - xtmp) / d31;
          pp_coefs_data[j2] = (xtmp - dnnm2) / d31;
          pp_coefs_data[(szs_idx_1 + j2) - 1] = 2.0 * dnnm2 - xtmp;
          pp_coefs_data[((szs_idx_1 - 1) << 1) + j2] = r;
          pp_coefs_data[3 * (szs_idx_1 - 1) + j2] = y_data[yoffset + j2];
        }
      }
      if (rtIsNaN(varargin_3)) {
        Vq = rtNaN;
      } else {
        j2 = pp_breaks_size_idx_1;
        low_i = 1;
        yoffset = 2;
        while (j2 > yoffset) {
          mid_i = (low_i >> 1) + (j2 >> 1);
          if (((low_i & 1) == 1) && ((j2 & 1) == 1)) {
            mid_i++;
          }
          if (varargin_3 >= pp_breaks_data[mid_i - 1]) {
            low_i = mid_i;
            yoffset = mid_i + 1;
          } else {
            j2 = mid_i;
          }
        }
        xtmp = varargin_3 - pp_breaks_data[low_i - 1];
        Vq = pp_coefs_data[low_i - 1];
        for (j2 = 2; j2 <= nx_tmp; j2++) {
          Vq = xtmp * Vq +
               pp_coefs_data[(low_i + (j2 - 1) * (pp_breaks_size_idx_1 - 1)) -
                             1];
        }
      }
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  return Vq;
}

/* End of code generation (interp1.c) */
