/*
 * EKF_UWB_SLAM_IMU_1.c
 *
 * Code generation for function 'EKF_UWB_SLAM_IMU_1'
 *
 */

/* Include files */
#include "EKF_UWB_SLAM_IMU_1.h"
#include "makePredA_3D_Simple.h"
#include "rt_nonfinite.h"
#include "xzgetrf.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Variable Definitions */
static double XhatUWB[15];

static double b_P[225];

static double Q[225];

static double R[36];

static double H[90];

static double IMUtimePrev;

/* Function Definitions */
double EKF_UWB_SLAM_IMU_1(const double acc[3], const double GyroD[3],
                          double IMUtime, const creal_T PosUWB, double HeadUWB,
                          double UWBtime, creal_T *PosHF, double *GammHF,
                          double *BetaHF)
{
  double b_A[225];
  double XhatIMU[15];
  double RotMat[9];
  double albega[3];
  double b_RotMat[3];
  double DiffTemp;
  double HeadingHF;
  double RotMat_tmp;
  double Vxyzb_idx_0;
  double Vxyzb_idx_1;
  double Vxyzb_idx_2;
  double b_RotMat_tmp;
  double bete;
  double c_RotMat_tmp;
  double d;
  double temp;
  int b_i;
  int i;
  int j;
  int jAcol;
  int jBcol;
  int k;
  DiffTemp = sin(XhatUWB[9]);
  RotMat_tmp = cos(XhatUWB[11]);
  temp = cos(XhatUWB[9]);
  b_RotMat_tmp = sin(XhatUWB[11]);
  c_RotMat_tmp = cos(XhatUWB[10]);
  bete = sin(XhatUWB[10]);
  RotMat[0] = DiffTemp * c_RotMat_tmp;
  Vxyzb_idx_0 = DiffTemp * bete;
  RotMat[3] = Vxyzb_idx_0 * b_RotMat_tmp + temp * RotMat_tmp;
  RotMat[6] = Vxyzb_idx_0 * RotMat_tmp - temp * b_RotMat_tmp;
  RotMat[1] = temp * c_RotMat_tmp;
  temp *= bete;
  RotMat[4] = temp * b_RotMat_tmp - DiffTemp * RotMat_tmp;
  RotMat[7] = temp * RotMat_tmp + DiffTemp * b_RotMat_tmp;
  RotMat[2] = -bete;
  RotMat[5] = c_RotMat_tmp * b_RotMat_tmp;
  RotMat[8] = c_RotMat_tmp * RotMat_tmp;
  DiffTemp = tan(XhatUWB[10]);
  Vxyzb_idx_0 = acc[0] - XhatUWB[6];
  Vxyzb_idx_1 = acc[1] - XhatUWB[7];
  Vxyzb_idx_2 = acc[2] - XhatUWB[8];
  for (i = 0; i < 3; i++) {
    d = RotMat[i];
    temp = RotMat[i + 3];
    bete = RotMat[i + 6];
    albega[i] = ((0.5 * d * Vxyzb_idx_0 + 0.5 * temp * Vxyzb_idx_1) +
                 0.5 * bete * Vxyzb_idx_2) *
                2.5E-5;
    b_RotMat[i] =
        ((d * Vxyzb_idx_0 + temp * Vxyzb_idx_1) + bete * Vxyzb_idx_2) * 0.005;
  }
  Vxyzb_idx_0 = b_RotMat[0] + XhatUWB[3];
  Vxyzb_idx_1 = b_RotMat[1] + XhatUWB[4];
  Vxyzb_idx_2 = b_RotMat[2] + XhatUWB[5];
  RotMat[0] = 1.0;
  RotMat[3] = b_RotMat_tmp * DiffTemp;
  RotMat[6] = RotMat_tmp * DiffTemp;
  RotMat[1] = 0.0;
  RotMat[4] = RotMat_tmp;
  RotMat[7] = -b_RotMat_tmp;
  RotMat[2] = 0.0;
  RotMat[5] = b_RotMat_tmp / c_RotMat_tmp;
  RotMat[8] = RotMat_tmp / c_RotMat_tmp;
  DiffTemp = GyroD[0] - XhatUWB[12];
  temp = GyroD[1] - XhatUWB[13];
  bete = GyroD[2] - XhatUWB[14];
  for (i = 0; i < 3; i++) {
    b_RotMat[i] =
        ((RotMat[i] * DiffTemp + RotMat[i + 3] * temp) + RotMat[i + 6] * bete) *
        0.005;
  }
  /*  %%% X Y change */
  /*  Pxb = Pxh + Vxh*dT +
   * 0.5*((acc(1)-abxh)*sin(alph)+(acc(2)-abyh)*cos(alph))*dT^2; */
  /*  Pyb = Pyh + Vyh*dT +
   * 0.5*((acc(1)-abxh)*cos(alph)+(acc(2)-abyh)*(-sin(alph)))*dT^2; */
  /*   */
  /*  Vxb = Vxh + ((acc(1)-abxh)*sin(alph)+(acc(2)-abyh)*cos(alph))*dT; */
  /*  Vyb = Vyh + ((acc(1)-abxh)*cos(alph)+(acc(2)-abyh)*(-sin(alph)))*dT; */
  /* %% X Y original */
  /*  Pxb = Pxh + Vxh*dT +
   * 0.5*((acc(1)-abxh)*cos(alph)*cos(betah)+(acc(2)-abyh)*(cos(alph)*sin(betah)*sin(gammah)-sin(alph)*cos(gammah))+(acc(3)-abzh)*(cos(alph)*sin(betah)*cos(gammah)+sin(alph)*sin(gammah)))*dT^2;
   */
  /*  Pyb = Pyh + Vyh*dT +
   * 0.5*((acc(1)-abxh)*sin(alph)*cos(betah)+(acc(2)-abyh)*(sin(alph)*sin(betah)*sin(gammah)+cos(alph)*cos(gammah))+(acc(3)-abzh)*(sin(alph)*sin(betah)*cos(gammah)-cos(alph)*sin(gammah)))*dT^2;
   */
  /*  Pzb = Pzh + Vzh*dT +
   * 0.5*(-1*(acc(1)-abxh)*sin(betah)+(acc(2)-abyh)*(cos(betah)*sin(gammah))+(acc(3)-abzh)*(cos(betah)*cos(gammah)))*dT^2;
   */
  /*   */
  /*  Vxb = Vxh +
   * ((acc(1)-abxh)*cos(alph)*cos(betah)+(acc(2)-abyh)*(cos(alph)*sin(betah)*sin(gammah)-sin(alph)*cos(gammah))+(acc(3)-abzh)*(cos(alph)*sin(betah)*cos(gammah)+sin(alph)*sin(gammah)))*dT;
   */
  /*  Vyb = Vyh +
   * ((acc(1)-abxh)*sin(alph)*cos(betah)+(acc(2)-abyh)*(sin(alph)*sin(betah)*sin(gammah)+cos(alph)*cos(gammah))+(acc(3)-abzh)*(sin(alph)*sin(betah)*cos(gammah)-cos(alph)*sin(gammah)))*dT;
   */
  /*  Vzb = Vzh +
   * (-1*(acc(1)-abxh)*sin(betah)+(acc(2)-abyh)*(cos(betah)*sin(gammah))+(acc(3)-abzh)*(cos(betah)*cos(gammah)))*dT;
   */
  /* %%%%%%% 발산 방지%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  if ((fabs(Vxyzb_idx_0) > 100.0) || (Vxyzb_idx_1 > 100.0) ||
      (Vxyzb_idx_2 > 100.0)) {
    memset(&XhatIMU[0], 0, 15U * sizeof(double));
  } else {
    XhatIMU[0] = (XhatUWB[0] + XhatUWB[3] * 0.005) + albega[0];
    XhatIMU[1] = (XhatUWB[1] + XhatUWB[4] * 0.005) + albega[1];
    XhatIMU[2] = (XhatUWB[2] + XhatUWB[5] * 0.005) + albega[2];
    XhatIMU[3] = Vxyzb_idx_0;
    XhatIMU[4] = Vxyzb_idx_1;
    XhatIMU[5] = Vxyzb_idx_2;
    XhatIMU[6] = XhatUWB[6];
    XhatIMU[7] = XhatUWB[7];
    XhatIMU[8] = XhatUWB[8];
    XhatIMU[9] = b_RotMat[2] + XhatUWB[9];
    XhatIMU[10] = b_RotMat[1] + XhatUWB[10];
    XhatIMU[11] = b_RotMat[0] + XhatUWB[11];
    XhatIMU[12] = XhatUWB[12];
    XhatIMU[13] = XhatUWB[13];
    XhatIMU[14] = XhatUWB[14];
  }
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Xbar = [Pxb Pyb Pzb Vxb Vyb Vzb abxb abyb abzb alpb betb gamb pbb qbb
   * rbb]; */
  if (IMUtimePrev - UWBtime < 0.0) {
    double A[225];
    double K[90];
    double K_tmp[90];
    double dv[90];
    double c_A[36];
    double b_HeadUWB[6];
    int ipiv[6];
    int b_K_tmp;
    int kBcol;
    boolean_T b[90];
    boolean_T exitg1;
    boolean_T y;
    makePredA_3D_Simple(XhatUWB, acc, GyroD, A);
    /*  [Zv,H] = EstEKF_Center_3D_Simple_1(Xbar); */
    DiffTemp = acc[1] - XhatIMU[7];
    temp = (acc[2] - 9.85) - XhatIMU[8];
    Vxyzb_idx_0 = atan(DiffTemp / temp);
    bete =
        atan((acc[0] - XhatIMU[6]) / sqrt(DiffTemp * DiffTemp + temp * temp));
    DiffTemp = XhatIMU[11] - Vxyzb_idx_0;
    if (rtIsNaN(DiffTemp)) {
      d = rtNaN;
    } else if (DiffTemp < 0.0) {
      d = -1.0;
    } else {
      d = (DiffTemp > 0.0);
    }
    Vxyzb_idx_0 += d * floor(fabs(DiffTemp) / 6.2831853071795862) * 2.0 *
                   3.1415926535897931;
    d = Vxyzb_idx_0 - XhatIMU[11];
    if (d > 3.1415926535897931) {
      Vxyzb_idx_0 -= 6.2831853071795862;
    } else if (d < -3.1415926535897931) {
      Vxyzb_idx_0 += 6.2831853071795862;
    }
    DiffTemp = XhatIMU[10] - bete;
    if (rtIsNaN(DiffTemp)) {
      d = rtNaN;
    } else if (DiffTemp < 0.0) {
      d = -1.0;
    } else {
      d = (DiffTemp > 0.0);
    }
    bete += d * floor(fabs(DiffTemp) / 6.2831853071795862) * 2.0 *
            3.1415926535897931;
    d = bete - XhatIMU[10];
    if (d > 3.1415926535897931) {
      bete -= 6.2831853071795862;
    } else if (d < -3.1415926535897931) {
      bete += 6.2831853071795862;
    }
    DiffTemp = XhatIMU[9] - HeadUWB;
    if (rtIsNaN(DiffTemp)) {
      d = rtNaN;
    } else if (DiffTemp < 0.0) {
      d = -1.0;
    } else {
      d = (DiffTemp > 0.0);
    }
    HeadUWB += d * floor(fabs(DiffTemp) / 6.2831853071795862) * 2.0 *
               3.1415926535897931;
    d = HeadUWB - XhatIMU[9];
    if (d > 3.1415926535897931) {
      HeadUWB -= 6.2831853071795862;
    } else if (d < -3.1415926535897931) {
      HeadUWB += 6.2831853071795862;
    }
    /*  game = AngleMatching(Xbar(3),game); */
    /*  bete = AngleMatching(Xbar(2),bete); */
    for (i = 0; i < 15; i++) {
      for (jAcol = 0; jAcol < 15; jAcol++) {
        d = 0.0;
        for (jBcol = 0; jBcol < 15; jBcol++) {
          d += A[i + 15 * jBcol] * b_P[jBcol + 15 * jAcol];
        }
        b_A[i + 15 * jAcol] = d;
      }
    }
    for (i = 0; i < 15; i++) {
      for (jAcol = 0; jAcol < 15; jAcol++) {
        d = 0.0;
        for (jBcol = 0; jBcol < 15; jBcol++) {
          d += b_A[i + 15 * jBcol] * A[jAcol + 15 * jBcol];
        }
        jBcol = i + 15 * jAcol;
        b_P[jBcol] = d + Q[jBcol];
      }
    }
    for (i = 0; i < 6; i++) {
      for (jAcol = 0; jAcol < 15; jAcol++) {
        K_tmp[jAcol + 15 * i] = H[i + 6 * jAcol];
      }
    }
    for (i = 0; i < 15; i++) {
      for (jAcol = 0; jAcol < 6; jAcol++) {
        d = 0.0;
        for (jBcol = 0; jBcol < 15; jBcol++) {
          d += b_P[i + 15 * jBcol] * K_tmp[jBcol + 15 * jAcol];
        }
        K[i + 15 * jAcol] = d;
      }
    }
    for (i = 0; i < 6; i++) {
      for (jAcol = 0; jAcol < 15; jAcol++) {
        d = 0.0;
        for (jBcol = 0; jBcol < 15; jBcol++) {
          d += H[i + 6 * jBcol] * b_P[jBcol + 15 * jAcol];
        }
        dv[i + 6 * jAcol] = d;
      }
      for (jAcol = 0; jAcol < 6; jAcol++) {
        d = 0.0;
        for (jBcol = 0; jBcol < 15; jBcol++) {
          d += dv[i + 6 * jBcol] * K_tmp[jBcol + 15 * jAcol];
        }
        jBcol = i + 6 * jAcol;
        c_A[jBcol] = d + R[jBcol];
      }
    }
    xzgetrf(c_A, ipiv);
    for (j = 0; j < 6; j++) {
      jBcol = 15 * j - 1;
      jAcol = 6 * j;
      for (k = 0; k < j; k++) {
        kBcol = 15 * k;
        d = c_A[k + jAcol];
        if (d != 0.0) {
          for (b_i = 0; b_i < 15; b_i++) {
            b_K_tmp = (b_i + jBcol) + 1;
            K[b_K_tmp] -= d * K[b_i + kBcol];
          }
        }
      }
      temp = 1.0 / c_A[j + jAcol];
      for (b_i = 0; b_i < 15; b_i++) {
        b_K_tmp = (b_i + jBcol) + 1;
        K[b_K_tmp] *= temp;
      }
    }
    for (j = 5; j >= 0; j--) {
      jBcol = 15 * j - 1;
      jAcol = 6 * j - 1;
      i = j + 2;
      for (k = i; k < 7; k++) {
        kBcol = 15 * (k - 1);
        d = c_A[k + jAcol];
        if (d != 0.0) {
          for (b_i = 0; b_i < 15; b_i++) {
            b_K_tmp = (b_i + jBcol) + 1;
            K[b_K_tmp] -= d * K[b_i + kBcol];
          }
        }
      }
    }
    for (j = 4; j >= 0; j--) {
      i = ipiv[j];
      if (i != j + 1) {
        for (b_i = 0; b_i < 15; b_i++) {
          jBcol = b_i + 15 * j;
          temp = K[jBcol];
          b_K_tmp = b_i + 15 * (i - 1);
          K[jBcol] = K[b_K_tmp];
          K[b_K_tmp] = temp;
        }
      }
    }
    b_HeadUWB[0] = HeadUWB;
    b_HeadUWB[1] = bete;
    b_HeadUWB[2] = Vxyzb_idx_0;
    b_HeadUWB[3] = PosUWB.re;
    b_HeadUWB[4] = PosUWB.im;
    b_HeadUWB[5] = 0.0;
    for (i = 0; i < 6; i++) {
      d = 0.0;
      for (jAcol = 0; jAcol < 15; jAcol++) {
        d += H[i + 6 * jAcol] * XhatIMU[jAcol];
      }
      b_HeadUWB[i] -= d;
    }
    for (i = 0; i < 15; i++) {
      d = 0.0;
      for (jAcol = 0; jAcol < 6; jAcol++) {
        d += K[i + 15 * jAcol] * b_HeadUWB[jAcol];
      }
      XhatUWB[i] = XhatIMU[i] + d;
      for (jAcol = 0; jAcol < 15; jAcol++) {
        d = 0.0;
        for (jBcol = 0; jBcol < 6; jBcol++) {
          d += K[i + 15 * jBcol] * H[jBcol + 6 * jAcol];
        }
        A[i + 15 * jAcol] = d;
      }
      for (jAcol = 0; jAcol < 15; jAcol++) {
        d = 0.0;
        for (jBcol = 0; jBcol < 15; jBcol++) {
          d += A[i + 15 * jBcol] * b_P[jBcol + 15 * jAcol];
        }
        jBcol = i + 15 * jAcol;
        b_A[jBcol] = b_P[jBcol] - d;
      }
    }
    memcpy(&b_P[0], &b_A[0], 225U * sizeof(double));
    for (i = 0; i < 90; i++) {
      b[i] = rtIsNaN(K[i]);
    }
    y = true;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 90)) {
      if (!b[k]) {
        y = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
    if (y) {
      PosHF->re = XhatIMU[0];
      PosHF->im = XhatIMU[1];
      HeadingHF = XhatIMU[9];
      *GammHF = XhatIMU[11];
      *BetaHF = XhatIMU[10];
    } else {
      PosHF->re = XhatUWB[0];
      PosHF->im = XhatUWB[1];
      HeadingHF = XhatUWB[9];
      *GammHF = XhatUWB[11];
      *BetaHF = XhatUWB[10];
    }
  } else {
    PosHF->re = XhatIMU[0];
    PosHF->im = XhatIMU[1];
    HeadingHF = XhatIMU[9];
    *GammHF = XhatIMU[11];
    *BetaHF = XhatIMU[10];
    memcpy(&XhatUWB[0], &XhatIMU[0], 15U * sizeof(double));
  }
  IMUtimePrev = IMUtime;
  return HeadingHF;
}

void EKF_UWB_SLAM_IMU_1_init(void)
{
  static const signed char iv1[225] = {
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  static const signed char iv[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                     0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                     0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
  static const signed char y[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  int Q_tmp;
  int i;
  for (i = 0; i < 36; i++) {
    R[i] = iv[i];
  }
  memset(&XhatUWB[0], 0, 15U * sizeof(double));
  for (i = 0; i < 225; i++) {
    Q[i] = iv1[i];
  }
  for (i = 0; i < 6; i++) {
    for (Q_tmp = 0; Q_tmp < 6; Q_tmp++) {
      Q[Q_tmp + 15 * i] = R[Q_tmp + 6 * i];
    }
  }
  for (i = 0; i < 3; i++) {
    Q_tmp = 15 * (i + 9);
    Q[Q_tmp + 9] = y[3 * i];
    Q[Q_tmp + 10] = y[3 * i + 1];
    Q[Q_tmp + 11] = y[3 * i + 2];
  }
  memset(&b_P[0], 0, 225U * sizeof(double));
  for (Q_tmp = 0; Q_tmp < 15; Q_tmp++) {
    b_P[Q_tmp + 15 * Q_tmp] = 1.0;
  }
  memset(&H[0], 0, 90U * sizeof(double));
  H[54] = 1.0;
  H[61] = 1.0;
  H[68] = 1.0;
  H[3] = 1.0;
  H[10] = 1.0;
  H[17] = 1.0;
  IMUtimePrev = 0.0;
}

/* End of code generation (EKF_UWB_SLAM_IMU_1.c) */
