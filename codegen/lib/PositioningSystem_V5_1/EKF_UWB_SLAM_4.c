/*
 * EKF_UWB_SLAM_4.c
 *
 * Code generation for function 'EKF_UWB_SLAM_4'
 *
 */

/* Include files */
#include "EKF_UWB_SLAM_4.h"
#include "PositioningSystem_V5_1_rtwutil.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include "xzgetrf.h"
#include <math.h>
#include <string.h>

/* Variable Definitions */
static double PosP[3];

static double roll;

static double pitch;

static double yaw;

static double s;

static double b_H[96];

static double c_P[256];

static double b_Q[256];

static double b_R[36];

static double Hroll;

static double Hpitch;

static double Hyaw;

static creal_T totPosP[5];

static double UWBless;

/* Function Definitions */
void EKF_UWB_SLAM_4(const creal_T PosUWB, double HeadUWB, double dPosSLAM[3],
                    double dHeadSLAM[3], const double PosSLAMc[3],
                    creal_T *Posn, double Headn[3])
{
  double A[256];
  double b[36];
  double xp[16];
  double b_xn_tmp_tmp;
  double b_xp_tmp;
  double b_xp_tmp_tmp;
  double c_xn_tmp_tmp;
  double c_xp_tmp;
  double c_xp_tmp_tmp;
  double d_xn_tmp_tmp;
  double d_xp_tmp;
  double e_xn_tmp_tmp;
  double e_xp_tmp;
  double f_xn_tmp_tmp;
  double f_xp_tmp;
  double g_xp_tmp;
  double h_xp_tmp;
  double i_xp_tmp;
  double j_xp_tmp;
  double k_xp_tmp;
  double l_xp_tmp;
  double m_xp_tmp;
  double n_xp_tmp;
  double o_xp_tmp;
  double p_xp_tmp;
  double xn_tmp_tmp;
  double xp_tmp;
  double xp_tmp_tmp;
  int b_i;
  int i;
  int i1;
  int j;
  int k;
  int pipk;
  boolean_T PosUWB_tmp;
  PosUWB_tmp = ((PosUWB.re != 0.0) || (PosUWB.im != 0.0));
  if (PosUWB_tmp) {
    if (PosP[0] == 0.0) {
      /* || (dPosSLAM(1)==0) */
      /*  PosP = [real(PosUWB);0;imag(PosUWB)]; */
      PosP[0] = PosSLAMc[0];
      PosP[1] = PosSLAMc[1];
      PosP[2] = PosSLAMc[2];
      s = 3.3027;
      roll = 0.034;
      pitch = 1.5508;
      yaw = 0.0076;
      /*  Eul = quat2eul(QuatInit); */
      Hroll = 0.034;
      /*  Hpitch = HeadUWB; */
      Hpitch = 1.5508;
      Hyaw = 0.0076;
      /*  s = 7.14; */
      /*  s = 8.3; */
    } else if (dPosSLAM[0] == 0.0) {
      /*  Eul = quat2eul(QuatInit); */
      /*  Hroll = Eul(1); */
      /*  Hpitch = Eul(2); */
      /*  Hyaw = Eul(3); */
      /*  load('MapParams4.mat') */
      s = 3.3027;
      roll = 0.034;
      pitch = 1.5508;
      yaw = 0.0076;
      /*  roll =  0; */
      /*  pitch = 0; */
      /*  yaw =  0; */
      /*          s = 4.6; */
      /*  sA = s*eulr2dcm([0 -pi*0.57 0]); */
      /*   */
      /*  SLAMposInit = [40.572;0;4.2114]; */
      /*  roll = 0; */
      /*  pitch = -pi*0.57; */
      /*  yaw = 0; */
      /*  load("MapParams3.mat"); */
      /*  s = sqrt(sum(abs(sA*[1;0;0]).^2)); */
      /*   */
      /*  TrMat = eulr2dcm([pi/2 0 pi/2]); */
      /*  Tempp = TrMat*sA/s; */
      /*  Tep = dcm2eulr(Tempp)-[pi/2; 0; pi/2]; */
      /*   */
      /*  % Tep = dcm2eulr(sA/s); */
      /*  roll = Tep(1); */
      /*  pitch = Tep(3); */
      /*  yaw = -Tep(2); */
      /*   */
      /*  Q = 1e-7*ones(16,16); */
      /*  Temp = Ajacob(ones(1,16)); */
      /*  Q(Temp==0)=0; */
      /*  roll = roll - dHeadSLAM(1); */
      /*  pitch = pitch - dHeadSLAM(2); */
      /*  yaw = yaw - dHeadSLAM(3); */
    } else if ((sqrt((dHeadSLAM[0] * dHeadSLAM[0] +
                      dHeadSLAM[1] * dHeadSLAM[1]) +
                     dHeadSLAM[2] * dHeadSLAM[2]) > 0.5) ||
               (sqrt((dPosSLAM[0] * dPosSLAM[0] + dPosSLAM[1] * dPosSLAM[1]) +
                     dPosSLAM[2] * dPosSLAM[2]) > 0.1)) {
      /*  s = 4.6; */
      /*  sA = s*eulr2dcm([0 -pi*0.57 0]); */
      /*   */
      /*  SLAMposInit = [40.572;0;4.2114]; */
      /*  roll = 0; */
      /*  pitch = -pi*0.57; */
      /*  yaw = 0; */
      dPosSLAM[0] = 0.0;
      dHeadSLAM[0] = 0.0;
      dPosSLAM[1] = 0.0;
      dHeadSLAM[1] = 0.0;
      dPosSLAM[2] = 0.0;
      dHeadSLAM[2] = 0.0;
      s = 3.3027;
      roll = 0.034;
      pitch = 1.5508;
      yaw = 0.0076;
      /*  PosP = PosSLAMc; */
    }
  }
  /* ------------------------ */
  xn_tmp_tmp = cos(roll);
  b_xn_tmp_tmp = sin(roll);
  c_xn_tmp_tmp = sin(yaw);
  d_xn_tmp_tmp = cos(yaw);
  e_xn_tmp_tmp = cos(pitch);
  f_xn_tmp_tmp = sin(pitch);
  xp_tmp = xn_tmp_tmp * f_xn_tmp_tmp;
  b_xp_tmp = xp_tmp * c_xn_tmp_tmp;
  c_xp_tmp = b_xn_tmp_tmp * c_xn_tmp_tmp;
  xp_tmp *= d_xn_tmp_tmp;
  d_xp_tmp = xn_tmp_tmp * e_xn_tmp_tmp;
  e_xp_tmp = b_xn_tmp_tmp * d_xn_tmp_tmp;
  f_xp_tmp = (((d_xp_tmp * dPosSLAM[0] - e_xp_tmp * dPosSLAM[1]) +
               b_xp_tmp * dPosSLAM[1]) +
              c_xp_tmp * dPosSLAM[2]) +
             xp_tmp * dPosSLAM[2];
  g_xp_tmp = s * f_xp_tmp;
  xp[0] = PosP[0] + g_xp_tmp;
  h_xp_tmp = b_xn_tmp_tmp * f_xn_tmp_tmp;
  xp_tmp_tmp = xn_tmp_tmp * d_xn_tmp_tmp;
  i_xp_tmp = xp_tmp_tmp * dPosSLAM[1];
  b_xp_tmp_tmp = h_xp_tmp * c_xn_tmp_tmp;
  j_xp_tmp = b_xp_tmp_tmp * dPosSLAM[1];
  k_xp_tmp = xn_tmp_tmp * c_xn_tmp_tmp * dPosSLAM[2];
  c_xp_tmp_tmp = h_xp_tmp * d_xn_tmp_tmp;
  h_xp_tmp = c_xp_tmp_tmp * dPosSLAM[2];
  l_xp_tmp = b_xn_tmp_tmp * e_xn_tmp_tmp;
  m_xp_tmp =
      (((l_xp_tmp * dPosSLAM[0] + i_xp_tmp) + j_xp_tmp) - k_xp_tmp) + h_xp_tmp;
  xp[1] = PosP[1] + s * m_xp_tmp;
  n_xp_tmp = e_xn_tmp_tmp * c_xn_tmp_tmp;
  o_xp_tmp = e_xn_tmp_tmp * d_xn_tmp_tmp;
  p_xp_tmp = (-f_xn_tmp_tmp * dPosSLAM[0] + n_xp_tmp * dPosSLAM[1]) +
             o_xp_tmp * dPosSLAM[2];
  xp[2] = PosP[2] + s * p_xp_tmp;
  xp[3] = dPosSLAM[0];
  xp[4] = dPosSLAM[1];
  xp[5] = dPosSLAM[2];
  xp[6] = roll;
  xp[7] = pitch;
  xp[8] = yaw;
  xp[9] = s;
  xp[10] =
      (Hroll + b_mod(dHeadSLAM[0] + 1.5707963267948966)) - 1.5707963267948966;
  xp[11] =
      (Hpitch + c_mod(dHeadSLAM[1] + 3.1415926535897931)) - 3.1415926535897931;
  xp[12] =
      (Hyaw + b_mod(dHeadSLAM[2] + 1.5707963267948966)) - 1.5707963267948966;
  xp[13] = dHeadSLAM[0];
  xp[14] = dHeadSLAM[1];
  xp[15] = dHeadSLAM[2];
  if (PosUWB_tmp) {
    double Pp[256];
    double b_A[256];
    double K[96];
    double b_Pp[96];
    double x[36];
    double b_PosUWB[6];
    int ipiv[6];
    int b_tmp;
    int kAcol;
    signed char p[6];
    /*  if (PosP(1)==0) */
    /*      PosP = [real(PosUWB);0;imag(PosUWB)]; */
    /*      pitch = HeadUWB; */
    /*      s = 8.3; */
    /*  end */
    /* ------------------------ */
    memset(&A[0], 0, 256U * sizeof(double));
    for (k = 0; k < 16; k++) {
      A[k + (k << 4)] = 1.0;
    }
    A[48] = s * xn_tmp_tmp * e_xn_tmp_tmp;
    A[64] = s * (-b_xn_tmp_tmp * d_xn_tmp_tmp + b_xp_tmp);
    A[80] = s * (c_xp_tmp + xp_tmp);
    A[96] =
        s *
        ((((-b_xn_tmp_tmp * e_xn_tmp_tmp * dPosSLAM[0] - i_xp_tmp) - j_xp_tmp) +
          k_xp_tmp) -
         h_xp_tmp);
    A[112] = s * ((xn_tmp_tmp * -f_xn_tmp_tmp * dPosSLAM[0] +
                   d_xp_tmp * c_xn_tmp_tmp * dPosSLAM[1]) +
                  d_xp_tmp * d_xn_tmp_tmp * dPosSLAM[2]);
    A[128] = s * (((c_xp_tmp * dPosSLAM[1] + xp_tmp * dPosSLAM[1]) +
                   e_xp_tmp * dPosSLAM[2]) -
                  b_xp_tmp * dPosSLAM[2]);
    A[144] = f_xp_tmp;
    A[49] = s * l_xp_tmp;
    A[65] = s * (xp_tmp_tmp + b_xp_tmp_tmp);
    xn_tmp_tmp = -xn_tmp_tmp * c_xn_tmp_tmp;
    A[81] = s * (xn_tmp_tmp + c_xp_tmp_tmp);
    A[97] = g_xp_tmp;
    A[113] = s * ((b_xn_tmp_tmp * -f_xn_tmp_tmp * dPosSLAM[0] +
                   l_xp_tmp * c_xn_tmp_tmp * dPosSLAM[1]) +
                  l_xp_tmp * d_xn_tmp_tmp * dPosSLAM[2]);
    A[129] = s * (((xn_tmp_tmp * dPosSLAM[1] + c_xp_tmp_tmp * dPosSLAM[1]) -
                   xp_tmp_tmp * dPosSLAM[2]) -
                  b_xp_tmp_tmp * dPosSLAM[2]);
    A[145] = m_xp_tmp;
    A[50] = s * -f_xn_tmp_tmp;
    A[66] = s * n_xp_tmp;
    A[82] = s * o_xp_tmp;
    A[114] = s * ((-e_xn_tmp_tmp * dPosSLAM[0] -
                   f_xn_tmp_tmp * c_xn_tmp_tmp * dPosSLAM[1]) -
                  f_xn_tmp_tmp * d_xn_tmp_tmp * dPosSLAM[2]);
    A[130] = s * (o_xp_tmp * dPosSLAM[1] - n_xp_tmp * dPosSLAM[2]);
    A[146] = p_xp_tmp;
    A[218] = 1.0;
    A[235] = 1.0;
    A[252] = 1.0;
    xn_tmp_tmp = HeadUWB - Hpitch;
    if (xn_tmp_tmp > 6.2831853071795862) {
      HeadUWB -= 6.2831853071795862 * floor(xn_tmp_tmp / 6.2831853071795862);
    } else {
      xn_tmp_tmp = Hpitch - HeadUWB;
      if (xn_tmp_tmp > 6.2831853071795862) {
        HeadUWB += 6.2831853071795862 * floor(xn_tmp_tmp / 6.2831853071795862);
      }
    }
    xn_tmp_tmp = HeadUWB - Hpitch;
    if (xn_tmp_tmp > 3.1415926535897931) {
      HeadUWB -= 6.2831853071795862;
    } else if (xn_tmp_tmp < -3.1415926535897931) {
      HeadUWB += 6.2831853071795862;
    }
    /*  z = [real(PosUWB);imag(PosUWB);HeadUWB]; */
    for (i = 0; i < 16; i++) {
      for (i1 = 0; i1 < 16; i1++) {
        xn_tmp_tmp = 0.0;
        for (pipk = 0; pipk < 16; pipk++) {
          xn_tmp_tmp += A[i + (pipk << 4)] * c_P[pipk + (i1 << 4)];
        }
        b_A[i + (i1 << 4)] = xn_tmp_tmp;
      }
      for (i1 = 0; i1 < 16; i1++) {
        xn_tmp_tmp = 0.0;
        for (pipk = 0; pipk < 16; pipk++) {
          kAcol = pipk << 4;
          xn_tmp_tmp += b_A[i + kAcol] * A[i1 + kAcol];
        }
        pipk = i + (i1 << 4);
        Pp[pipk] = xn_tmp_tmp + b_Q[pipk];
      }
    }
    for (i = 0; i < 6; i++) {
      for (i1 = 0; i1 < 16; i1++) {
        K[i1 + (i << 4)] = b_H[i + 6 * i1];
      }
    }
    memset(&b[0], 0, 36U * sizeof(double));
    for (i = 0; i < 6; i++) {
      for (i1 = 0; i1 < 16; i1++) {
        xn_tmp_tmp = 0.0;
        for (pipk = 0; pipk < 16; pipk++) {
          xn_tmp_tmp += b_H[i + 6 * pipk] * Pp[pipk + (i1 << 4)];
        }
        b_Pp[i + 6 * i1] = xn_tmp_tmp;
      }
      for (i1 = 0; i1 < 6; i1++) {
        xn_tmp_tmp = 0.0;
        for (pipk = 0; pipk < 16; pipk++) {
          xn_tmp_tmp += b_Pp[i + 6 * pipk] * K[pipk + (i1 << 4)];
        }
        pipk = i + 6 * i1;
        x[pipk] = xn_tmp_tmp + b_R[pipk];
      }
    }
    xzgetrf(x, ipiv);
    for (i = 0; i < 6; i++) {
      p[i] = (signed char)(i + 1);
    }
    for (k = 0; k < 5; k++) {
      i = ipiv[k];
      if (i > k + 1) {
        pipk = p[i - 1];
        p[i - 1] = p[k];
        p[k] = (signed char)pipk;
      }
    }
    for (k = 0; k < 6; k++) {
      b_tmp = 6 * (p[k] - 1);
      b[k + b_tmp] = 1.0;
      for (j = k + 1; j < 7; j++) {
        i = (j + b_tmp) - 1;
        if (b[i] != 0.0) {
          i1 = j + 1;
          for (b_i = i1; b_i < 7; b_i++) {
            pipk = (b_i + b_tmp) - 1;
            b[pipk] -= b[i] * x[(b_i + 6 * (j - 1)) - 1];
          }
        }
      }
    }
    for (j = 0; j < 6; j++) {
      pipk = 6 * j;
      for (k = 5; k >= 0; k--) {
        kAcol = 6 * k;
        i = k + pipk;
        xn_tmp_tmp = b[i];
        if (xn_tmp_tmp != 0.0) {
          b[i] = xn_tmp_tmp / x[k + kAcol];
          for (b_i = 0; b_i < k; b_i++) {
            b_tmp = b_i + pipk;
            b[b_tmp] -= b[i] * x[b_i + kAcol];
          }
        }
      }
    }
    for (i = 0; i < 16; i++) {
      for (i1 = 0; i1 < 6; i1++) {
        xn_tmp_tmp = 0.0;
        for (pipk = 0; pipk < 16; pipk++) {
          xn_tmp_tmp += Pp[i + (pipk << 4)] * K[pipk + (i1 << 4)];
        }
        b_Pp[i + (i1 << 4)] = xn_tmp_tmp;
      }
    }
    for (i = 0; i < 16; i++) {
      for (i1 = 0; i1 < 6; i1++) {
        xn_tmp_tmp = 0.0;
        for (pipk = 0; pipk < 6; pipk++) {
          xn_tmp_tmp += b_Pp[i + (pipk << 4)] * b[pipk + 6 * i1];
        }
        K[i + (i1 << 4)] = xn_tmp_tmp;
      }
    }
    b_PosUWB[0] = PosUWB.re;
    b_PosUWB[1] = 0.0;
    b_PosUWB[2] = PosUWB.im;
    b_PosUWB[3] = 0.0;
    b_PosUWB[4] = HeadUWB;
    b_PosUWB[5] = 0.0;
    for (i = 0; i < 6; i++) {
      xn_tmp_tmp = 0.0;
      for (i1 = 0; i1 < 16; i1++) {
        xn_tmp_tmp += b_H[i + 6 * i1] * xp[i1];
      }
      b_PosUWB[i] -= xn_tmp_tmp;
    }
    for (i = 0; i < 16; i++) {
      xn_tmp_tmp = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        xn_tmp_tmp += K[i + (i1 << 4)] * b_PosUWB[i1];
      }
      xp[i] += xn_tmp_tmp;
      for (i1 = 0; i1 < 16; i1++) {
        xn_tmp_tmp = 0.0;
        for (pipk = 0; pipk < 6; pipk++) {
          xn_tmp_tmp += K[i + (pipk << 4)] * b_H[pipk + 6 * i1];
        }
        A[i + (i1 << 4)] = xn_tmp_tmp;
      }
      for (i1 = 0; i1 < 16; i1++) {
        xn_tmp_tmp = 0.0;
        for (pipk = 0; pipk < 16; pipk++) {
          xn_tmp_tmp += A[i + (pipk << 4)] * Pp[pipk + (i1 << 4)];
        }
        pipk = i + (i1 << 4);
        c_P[pipk] = Pp[pipk] - xn_tmp_tmp;
      }
    }
    UWBless = 0.0;
  } else {
    UWBless++;
    /*  TempAA = eulr2dcm(HeadSLAMc)*[1;0;0]; */
    /*  Eulr_r = quaternionToEulerYXZ(QuatInit, eulr2dcm([roll  pitch  yaw]));
     */
    /*  x(1) = PosSLAMc(1); */
    /*  x(2) = PosSLAMc(2); */
    /*  x(3) = PosSLAMc(3); */
    /*  x(11) = 0; */
    /*  x(12) = atan2(TempAA(1),TempAA(3)); */
    /*  x(13) = 0; */
    /*  x(1) = PosSLAMc(1); */
    /*  x(2) = PosSLAMc(2); */
    /*  x(3) = PosSLAMc(3); */
    /*   */
    /*  x(11) = HeadSLAMc(1); */
    /*  x(12) = HeadSLAMc(2); */
    /*  x(13) = HeadSLAMc(3); */
    if (UWBless > 30.0) {
      /*  x(11:13) = Eulr_r; */
      xp[11] = c_mod(rt_atan2d_snf(totPosP[4].re - totPosP[0].re,
                                   totPosP[4].im - totPosP[0].im) +
                     6.2831853071795862);
    }
  }
  PosP[0] = xp[0];
  PosP[1] = xp[1];
  PosP[2] = xp[2];
  roll = xp[6];
  pitch = xp[7];
  yaw = xp[8];
  s = xp[9];
  /*   */
  /*  Tep = dcm2eulr(sA); */
  /*  roll = Tep(1); */
  /*  pitch = -Tep(3); */
  /*  yaw = Tep(2); */
  Hroll = xp[10];
  Hpitch = xp[11];
  Hyaw = xp[12];
  xn_tmp_tmp = PosP[0] + PosP[2] * 0.0;
  Posn->re = xn_tmp_tmp;
  Posn->im = PosP[2];
  Headn[0] = Hroll;
  Headn[1] = Hpitch;
  Headn[2] = Hyaw;
  /*   */
  /*  totHead = [totHead [Hroll;Hpitch;Hyaw]]; */
  /*  totA = [totA [roll;pitch;yaw]]; */
  /*  totHUWB = [totHUWB HeadUWB]; */
  /*   */
  totPosP[0] = totPosP[1];
  totPosP[1] = totPosP[2];
  totPosP[2] = totPosP[3];
  totPosP[3] = totPosP[4];
  totPosP[4].re = xn_tmp_tmp;
  totPosP[4].im = PosP[2];
  /*  totPosUWB = [totPosUWB PosUWB]; */
  /*   */
  /*  totdHeadSLAM = [totdHeadSLAM reshape(dHeadSLAM(:),3,1)]; */
  /*  totdPosSLAM = [totdPosSLAM dPosSLAM]; */
}

void EKF_UWB_SLAM_4_init(void)
{
  static const unsigned char uv[217] = {
      1U,   2U,   3U,   4U,   5U,   6U,   7U,   8U,   9U,   10U,  11U,  12U,
      13U,  14U,  15U,  16U,  18U,  19U,  20U,  21U,  22U,  23U,  24U,  25U,
      26U,  27U,  28U,  29U,  30U,  31U,  32U,  33U,  35U,  36U,  37U,  38U,
      39U,  40U,  41U,  42U,  43U,  44U,  45U,  46U,  47U,  52U,  53U,  54U,
      55U,  56U,  57U,  58U,  59U,  60U,  61U,  62U,  63U,  67U,  69U,  70U,
      71U,  72U,  73U,  74U,  75U,  76U,  77U,  78U,  79U,  83U,  84U,  86U,
      87U,  88U,  89U,  90U,  91U,  92U,  93U,  94U,  95U,  98U,  99U,  100U,
      101U, 103U, 104U, 105U, 106U, 107U, 108U, 109U, 110U, 111U, 115U, 116U,
      117U, 118U, 120U, 121U, 122U, 123U, 124U, 125U, 126U, 127U, 131U, 132U,
      133U, 134U, 135U, 137U, 138U, 139U, 140U, 141U, 142U, 143U, 147U, 148U,
      149U, 150U, 151U, 152U, 154U, 155U, 156U, 157U, 158U, 159U, 160U, 161U,
      162U, 163U, 164U, 165U, 166U, 167U, 168U, 169U, 171U, 172U, 173U, 174U,
      175U, 176U, 177U, 178U, 179U, 180U, 181U, 182U, 183U, 184U, 185U, 186U,
      188U, 189U, 190U, 191U, 192U, 193U, 194U, 195U, 196U, 197U, 198U, 199U,
      200U, 201U, 202U, 203U, 205U, 206U, 207U, 208U, 209U, 210U, 211U, 212U,
      213U, 214U, 215U, 216U, 217U, 219U, 220U, 222U, 223U, 224U, 225U, 226U,
      227U, 228U, 229U, 230U, 231U, 232U, 233U, 234U, 236U, 237U, 239U, 240U,
      241U, 242U, 243U, 244U, 245U, 246U, 247U, 248U, 249U, 250U, 251U, 253U,
      254U};
  static const signed char iv[96] = {
      1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static const signed char iv1[36] = {
      100, 0, 0, 0,   0, 0, 0, 100, 0, 0, 0,   0, 0, 0, 100, 0, 0, 0,
      0,   0, 0, 100, 0, 0, 0, 0,   0, 0, 100, 0, 0, 0, 0,   0, 0, 100};
  int i;
  for (i = 0; i < 96; i++) {
    b_H[i] = iv[i];
  }
  for (i = 0; i < 36; i++) {
    b_R[i] = iv1[i];
  }
  Hroll = 0.0;
  Hpitch = 0.0;
  Hyaw = 0.0;
  UWBless = 0.0;
  memset(&totPosP[0], 0, 5U * sizeof(creal_T));
  /*  s = 8.14; */
  PosP[0] = 0.0;
  PosP[1] = 0.0;
  PosP[2] = 0.0;
  for (i = 0; i < 256; i++) {
    b_Q[i] = 1.0E-5;
  }
  /*  Q = 1e-7*ones(16,16); */
  for (i = 0; i < 217; i++) {
    b_Q[uv[i]] = 0.0;
  }
  for (i = 0; i < 256; i++) {
    c_P[i] = 1.0;
  }
  for (i = 0; i < 217; i++) {
    c_P[uv[i]] = 0.0;
  }
  /*   */
  /*      ParkingLot = uint8([]); */
  /*      ParkingLotS= imread('B1_ParkingLot.jpg'); */
  /*      for sx = 1:size(ParkingLotS,1) */
  /*          for sy = 1:size(ParkingLotS,2) */
  /*              % ParkingLot(sy,sx,:) =
   * ParkingLotS(size(ParkingLotS,1)-sx+1,size(ParkingLotS,2)-sy+1,:); */
  /*              ParkingLot(sy,sx,:) =
   * ParkingLotS(sx,size(ParkingLotS,2)-sy+1,:); */
  /*          end */
  /*      end */
  /*      figure(123411);hold off; */
  /*      imshow(ParkingLot); */
  /*  load('MapParams4.mat') */
  s = 3.3027;
  roll = 0.034;
  pitch = 1.5508;
  yaw = 0.0076;
}

void b_EKF_UWB_SLAM_4(const double dPosSLAM[3], const double dHeadSLAM[3],
                      creal_T *Posn, double Headn[3])
{
  double PosP_tmp;
  double b_xn_tmp;
  double c_xn_tmp;
  double d_xn_tmp;
  double e_xn_tmp;
  double f_xn_tmp;
  double xn_tmp;
  /* ------------------------ */
  xn_tmp = cos(roll);
  b_xn_tmp = sin(roll);
  c_xn_tmp = sin(yaw);
  d_xn_tmp = cos(yaw);
  e_xn_tmp = cos(pitch);
  f_xn_tmp = sin(pitch);
  Hpitch =
      (Hpitch + c_mod(dHeadSLAM[1] + 3.1415926535897931)) - 3.1415926535897931;
  UWBless++;
  /*  TempAA = eulr2dcm(HeadSLAMc)*[1;0;0]; */
  /*  Eulr_r = quaternionToEulerYXZ(QuatInit, eulr2dcm([roll  pitch  yaw])); */
  /*  x(1) = PosSLAMc(1); */
  /*  x(2) = PosSLAMc(2); */
  /*  x(3) = PosSLAMc(3); */
  /*  x(11) = 0; */
  /*  x(12) = atan2(TempAA(1),TempAA(3)); */
  /*  x(13) = 0; */
  /*  x(1) = PosSLAMc(1); */
  /*  x(2) = PosSLAMc(2); */
  /*  x(3) = PosSLAMc(3); */
  /*   */
  /*  x(11) = HeadSLAMc(1); */
  /*  x(12) = HeadSLAMc(2); */
  /*  x(13) = HeadSLAMc(3); */
  if (UWBless > 30.0) {
    /*  x(11:13) = Eulr_r; */
    Hpitch = c_mod(rt_atan2d_snf(totPosP[4].re - totPosP[0].re,
                                 totPosP[4].im - totPosP[0].im) +
                   6.2831853071795862);
  }
  PosP_tmp = xn_tmp * f_xn_tmp;
  PosP[0] +=
      s *
      ((((xn_tmp * e_xn_tmp * dPosSLAM[0] - b_xn_tmp * d_xn_tmp * dPosSLAM[1]) +
         PosP_tmp * c_xn_tmp * dPosSLAM[1]) +
        b_xn_tmp * c_xn_tmp * dPosSLAM[2]) +
       PosP_tmp * d_xn_tmp * dPosSLAM[2]);
  PosP_tmp = b_xn_tmp * f_xn_tmp;
  PosP[1] +=
      s *
      ((((b_xn_tmp * e_xn_tmp * dPosSLAM[0] + xn_tmp * d_xn_tmp * dPosSLAM[1]) +
         PosP_tmp * c_xn_tmp * dPosSLAM[1]) -
        xn_tmp * c_xn_tmp * dPosSLAM[2]) +
       PosP_tmp * d_xn_tmp * dPosSLAM[2]);
  PosP[2] +=
      s * ((-f_xn_tmp * dPosSLAM[0] + e_xn_tmp * c_xn_tmp * dPosSLAM[1]) +
           e_xn_tmp * d_xn_tmp * dPosSLAM[2]);
  /*   */
  /*  Tep = dcm2eulr(sA); */
  /*  roll = Tep(1); */
  /*  pitch = -Tep(3); */
  /*  yaw = Tep(2); */
  Hroll =
      (Hroll + b_mod(dHeadSLAM[0] + 1.5707963267948966)) - 1.5707963267948966;
  Hyaw = (Hyaw + b_mod(dHeadSLAM[2] + 1.5707963267948966)) - 1.5707963267948966;
  xn_tmp = PosP[0] + PosP[2] * 0.0;
  Posn->re = xn_tmp;
  Posn->im = PosP[2];
  Headn[0] = Hroll;
  Headn[1] = Hpitch;
  Headn[2] = Hyaw;
  /*   */
  /*  totHead = [totHead [Hroll;Hpitch;Hyaw]]; */
  /*  totA = [totA [roll;pitch;yaw]]; */
  /*  totHUWB = [totHUWB HeadUWB]; */
  /*   */
  totPosP[0] = totPosP[1];
  totPosP[1] = totPosP[2];
  totPosP[2] = totPosP[3];
  totPosP[3] = totPosP[4];
  totPosP[4].re = xn_tmp;
  totPosP[4].im = PosP[2];
  /*  totPosUWB = [totPosUWB PosUWB]; */
  /*   */
  /*  totdHeadSLAM = [totdHeadSLAM reshape(dHeadSLAM(:),3,1)]; */
  /*  totdPosSLAM = [totdPosSLAM dPosSLAM]; */
}

/* End of code generation (EKF_UWB_SLAM_4.c) */
