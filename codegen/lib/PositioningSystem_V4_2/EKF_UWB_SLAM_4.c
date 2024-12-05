/*
 * EKF_UWB_SLAM_4.c
 *
 * Code generation for function 'EKF_UWB_SLAM_4'
 *
 */

/* Include files */
#include "EKF_UWB_SLAM_4.h"
#include "PositioningSystem_V4_2_data.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Variable Definitions */
static double H[96];

static double b_P[256];

static double Q[256];

static double R[36];

/* Function Definitions */
void EKF_UWB_SLAM_4(const creal_T PosUWB, double HeadUWB, double dPosSLAM[3],
                    double dHeadSLAM[3], const double PosSLAMc[3],
                    const double HeadSLAMc[3], creal_T *Posn, double Headn[3])
{
  double A[256];
  double y[36];
  double xhat[16];
  double xp[16];
  int b_i;
  int i;
  int j;
  int jp1j;
  int k;
  int kAcol;
  int x_tmp;
  boolean_T PosUWB_tmp;
  PosUWB_tmp = ((PosUWB.re != 0.0) || (PosUWB.im != 0.0));
  if (PosUWB_tmp) {
    if (PosP[0] == 0.0) {
      /* || (dPosSLAM(1)==0) */
      /*  PosP = [real(PosUWB);0;imag(PosUWB)]; */
      PosP[0] = PosSLAMc[0];
      PosP[1] = PosSLAMc[1];
      PosP[2] = PosSLAMc[2];
      /*  Eul = quat2eul(QuatInit); */
      Hroll = HeadSLAMc[0];
      /*  Hpitch = HeadUWB; */
      Hpitch = HeadSLAMc[1];
      Hyaw = HeadSLAMc[2];
      /*  s = 7.14; */
      /*  s = 8.3; */
    } else if (dPosSLAM[0] == 0.0) {
      /*  Eul = quat2eul(QuatInit); */
      /*  Hroll = Eul(1); */
      /*  Hpitch = Eul(2); */
      /*  Hyaw = Eul(3); */
      /*  load('MapParams4.mat') */
      s = 4.5839;
      roll = -0.006;
      pitch = -4.4647;
      yaw = -0.0458;
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
      s = 4.5839;
      roll = -0.006;
      pitch = -4.4647;
      yaw = -0.0458;
      /*  PosP = PosSLAMc; */
    }
  }
  xhat[0] = PosP[0];
  xhat[1] = PosP[1];
  xhat[2] = PosP[2];
  xhat[3] = dPosSLAM[0];
  xhat[4] = dPosSLAM[1];
  xhat[5] = dPosSLAM[2];
  xhat[6] = roll;
  xhat[7] = pitch;
  xhat[8] = yaw;
  xhat[9] = s;
  xhat[10] = Hroll;
  xhat[11] = Hpitch;
  xhat[12] = Hyaw;
  xhat[13] = dHeadSLAM[0];
  xhat[14] = dHeadSLAM[1];
  xhat[15] = dHeadSLAM[2];
  fx(xhat, xp);
  if (PosUWB_tmp) {
    double Pp[256];
    double b_A[256];
    double K[96];
    double b_Pp[96];
    double x[36];
    double b_PosUWB[6];
    double smax;
    int jA;
    signed char ipiv[6];
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
    double A_tmp;
    double A_tmp_tmp;
    double b_A_tmp;
    double c_A_tmp;
    double d_A_tmp;
    double e_A_tmp;
    double f_A_tmp;
    double g_A_tmp;
    double h_A_tmp;
    double i_A_tmp;
    double j_A_tmp;
    double k_A_tmp;
    double l_A_tmp;
    double m_A_tmp;
    double n_A_tmp;
    double o_A_tmp;
    double p_A_tmp;
    A_tmp = cos(roll);
    b_A_tmp = cos(pitch);
    A[48] = s * A_tmp * b_A_tmp;
    A_tmp_tmp = sin(roll);
    c_A_tmp = sin(yaw);
    d_A_tmp = cos(yaw);
    smax = sin(pitch);
    e_A_tmp = A_tmp * smax;
    f_A_tmp = e_A_tmp * c_A_tmp;
    A[64] = s * (-A_tmp_tmp * d_A_tmp + f_A_tmp);
    g_A_tmp = A_tmp_tmp * c_A_tmp;
    e_A_tmp *= d_A_tmp;
    A[80] = s * (g_A_tmp + e_A_tmp);
    h_A_tmp = A_tmp_tmp * smax;
    i_A_tmp = A_tmp * d_A_tmp;
    j_A_tmp = h_A_tmp * c_A_tmp;
    h_A_tmp *= d_A_tmp;
    k_A_tmp = i_A_tmp * dPosSLAM[1];
    l_A_tmp = j_A_tmp * dPosSLAM[1];
    m_A_tmp = A_tmp * c_A_tmp * dPosSLAM[2];
    n_A_tmp = h_A_tmp * dPosSLAM[2];
    A[96] = s * ((((-A_tmp_tmp * b_A_tmp * dPosSLAM[0] - k_A_tmp) - l_A_tmp) +
                  m_A_tmp) -
                 n_A_tmp);
    o_A_tmp = A_tmp * b_A_tmp;
    A[112] =
        s * ((A_tmp * -smax * dPosSLAM[0] + o_A_tmp * c_A_tmp * dPosSLAM[1]) +
             o_A_tmp * d_A_tmp * dPosSLAM[2]);
    p_A_tmp = A_tmp_tmp * d_A_tmp;
    A[128] = s * (((g_A_tmp * dPosSLAM[1] + e_A_tmp * dPosSLAM[1]) +
                   p_A_tmp * dPosSLAM[2]) -
                  f_A_tmp * dPosSLAM[2]);
    e_A_tmp = (((o_A_tmp * dPosSLAM[0] - p_A_tmp * dPosSLAM[1]) +
                f_A_tmp * dPosSLAM[1]) +
               g_A_tmp * dPosSLAM[2]) +
              e_A_tmp * dPosSLAM[2];
    A[144] = e_A_tmp;
    f_A_tmp = A_tmp_tmp * b_A_tmp;
    A[49] = s * f_A_tmp;
    A[65] = s * (i_A_tmp + j_A_tmp);
    A_tmp = -A_tmp * c_A_tmp;
    A[81] = s * (A_tmp + h_A_tmp);
    A[97] = s * e_A_tmp;
    A[113] =
        s *
        ((A_tmp_tmp * -smax * dPosSLAM[0] + f_A_tmp * c_A_tmp * dPosSLAM[1]) +
         f_A_tmp * d_A_tmp * dPosSLAM[2]);
    A[129] = s * (((A_tmp * dPosSLAM[1] + h_A_tmp * dPosSLAM[1]) -
                   i_A_tmp * dPosSLAM[2]) -
                  j_A_tmp * dPosSLAM[2]);
    A[145] =
        (((f_A_tmp * dPosSLAM[0] + k_A_tmp) + l_A_tmp) - m_A_tmp) + n_A_tmp;
    A[50] = s * -smax;
    A_tmp = b_A_tmp * c_A_tmp;
    A[66] = s * A_tmp;
    e_A_tmp = b_A_tmp * d_A_tmp;
    A[82] = s * e_A_tmp;
    A[114] = s * ((-b_A_tmp * dPosSLAM[0] - smax * c_A_tmp * dPosSLAM[1]) -
                  smax * d_A_tmp * dPosSLAM[2]);
    A[130] = s * (e_A_tmp * dPosSLAM[1] - A_tmp * dPosSLAM[2]);
    A[146] =
        (-smax * dPosSLAM[0] + A_tmp * dPosSLAM[1]) + e_A_tmp * dPosSLAM[2];
    A[218] = 1.0;
    A[235] = 1.0;
    A[252] = 1.0;
    smax = HeadUWB - Hpitch;
    if (smax > 6.2831853071795862) {
      HeadUWB -= 6.2831853071795862 * floor(smax / 6.2831853071795862);
    } else {
      smax = Hpitch - HeadUWB;
      if (smax > 6.2831853071795862) {
        HeadUWB += 6.2831853071795862 * floor(smax / 6.2831853071795862);
      }
    }
    smax = HeadUWB - Hpitch;
    if (smax > 3.1415926535897931) {
      HeadUWB -= 6.2831853071795862;
    } else if (smax < -3.1415926535897931) {
      HeadUWB += 6.2831853071795862;
    }
    /*  z = [real(PosUWB);imag(PosUWB);HeadUWB]; */
    for (i = 0; i < 16; i++) {
      for (jp1j = 0; jp1j < 16; jp1j++) {
        smax = 0.0;
        for (kAcol = 0; kAcol < 16; kAcol++) {
          smax += A[i + (kAcol << 4)] * b_P[kAcol + (jp1j << 4)];
        }
        b_A[i + (jp1j << 4)] = smax;
      }
      for (jp1j = 0; jp1j < 16; jp1j++) {
        smax = 0.0;
        for (kAcol = 0; kAcol < 16; kAcol++) {
          jA = kAcol << 4;
          smax += b_A[i + jA] * A[jp1j + jA];
        }
        jA = i + (jp1j << 4);
        Pp[jA] = smax + Q[jA];
      }
    }
    for (i = 0; i < 6; i++) {
      for (jp1j = 0; jp1j < 16; jp1j++) {
        jA = i + 6 * jp1j;
        K[jp1j + (i << 4)] = H[jA];
        smax = 0.0;
        for (kAcol = 0; kAcol < 16; kAcol++) {
          smax += H[i + 6 * kAcol] * Pp[kAcol + (jp1j << 4)];
        }
        b_Pp[jA] = smax;
      }
    }
    for (i = 0; i < 6; i++) {
      for (jp1j = 0; jp1j < 6; jp1j++) {
        smax = 0.0;
        for (kAcol = 0; kAcol < 16; kAcol++) {
          smax += b_Pp[i + 6 * kAcol] * K[kAcol + (jp1j << 4)];
        }
        x_tmp = i + 6 * jp1j;
        x[x_tmp] = smax + R[x_tmp];
      }
    }
    memset(&y[0], 0, 36U * sizeof(double));
    for (i = 0; i < 6; i++) {
      ipiv[i] = (signed char)(i + 1);
    }
    for (j = 0; j < 5; j++) {
      int b_tmp;
      int mmj_tmp;
      mmj_tmp = 4 - j;
      b_tmp = j * 7;
      jp1j = b_tmp + 2;
      jA = 6 - j;
      kAcol = 0;
      smax = fabs(x[b_tmp]);
      for (k = 2; k <= jA; k++) {
        s = fabs(x[(b_tmp + k) - 1]);
        if (s > smax) {
          kAcol = k - 1;
          smax = s;
        }
      }
      if (x[b_tmp + kAcol] != 0.0) {
        if (kAcol != 0) {
          jA = j + kAcol;
          ipiv[j] = (signed char)(jA + 1);
          for (k = 0; k < 6; k++) {
            kAcol = j + k * 6;
            smax = x[kAcol];
            x_tmp = jA + k * 6;
            x[kAcol] = x[x_tmp];
            x[x_tmp] = smax;
          }
        }
        i = (b_tmp - j) + 6;
        for (b_i = jp1j; b_i <= i; b_i++) {
          x[b_i - 1] /= x[b_tmp];
        }
      }
      jA = b_tmp;
      for (kAcol = 0; kAcol <= mmj_tmp; kAcol++) {
        smax = x[(b_tmp + kAcol * 6) + 6];
        if (smax != 0.0) {
          i = jA + 8;
          jp1j = (jA - j) + 12;
          for (x_tmp = i; x_tmp <= jp1j; x_tmp++) {
            x[x_tmp - 1] += x[((b_tmp + x_tmp) - jA) - 7] * -smax;
          }
        }
        jA += 6;
      }
    }
    for (i = 0; i < 6; i++) {
      p[i] = (signed char)(i + 1);
    }
    for (k = 0; k < 5; k++) {
      signed char i1;
      i1 = ipiv[k];
      if (i1 > k + 1) {
        jA = p[i1 - 1];
        p[i1 - 1] = p[k];
        p[k] = (signed char)jA;
      }
    }
    for (k = 0; k < 6; k++) {
      x_tmp = 6 * (p[k] - 1);
      y[k + x_tmp] = 1.0;
      for (j = k + 1; j < 7; j++) {
        i = (j + x_tmp) - 1;
        if (y[i] != 0.0) {
          jp1j = j + 1;
          for (b_i = jp1j; b_i < 7; b_i++) {
            jA = (b_i + x_tmp) - 1;
            y[jA] -= y[i] * x[(b_i + 6 * (j - 1)) - 1];
          }
        }
      }
    }
    for (j = 0; j < 6; j++) {
      jA = 6 * j;
      for (k = 5; k >= 0; k--) {
        kAcol = 6 * k;
        i = k + jA;
        smax = y[i];
        if (smax != 0.0) {
          y[i] = smax / x[k + kAcol];
          for (b_i = 0; b_i < k; b_i++) {
            x_tmp = b_i + jA;
            y[x_tmp] -= y[i] * x[b_i + kAcol];
          }
        }
      }
    }
    for (i = 0; i < 16; i++) {
      for (jp1j = 0; jp1j < 6; jp1j++) {
        smax = 0.0;
        for (kAcol = 0; kAcol < 16; kAcol++) {
          smax += Pp[i + (kAcol << 4)] * K[kAcol + (jp1j << 4)];
        }
        b_Pp[i + (jp1j << 4)] = smax;
      }
    }
    for (i = 0; i < 16; i++) {
      for (jp1j = 0; jp1j < 6; jp1j++) {
        smax = 0.0;
        for (kAcol = 0; kAcol < 6; kAcol++) {
          smax += b_Pp[i + (kAcol << 4)] * y[kAcol + 6 * jp1j];
        }
        K[i + (jp1j << 4)] = smax;
      }
    }
    b_PosUWB[0] = PosUWB.re;
    b_PosUWB[1] = 0.0;
    b_PosUWB[2] = PosUWB.im;
    b_PosUWB[3] = 0.0;
    b_PosUWB[4] = HeadUWB;
    b_PosUWB[5] = 0.0;
    for (i = 0; i < 6; i++) {
      smax = 0.0;
      for (jp1j = 0; jp1j < 16; jp1j++) {
        smax += H[i + 6 * jp1j] * xp[jp1j];
      }
      b_PosUWB[i] -= smax;
    }
    for (i = 0; i < 16; i++) {
      smax = 0.0;
      for (jp1j = 0; jp1j < 6; jp1j++) {
        smax += K[i + (jp1j << 4)] * b_PosUWB[jp1j];
      }
      xp[i] += smax;
      for (jp1j = 0; jp1j < 16; jp1j++) {
        smax = 0.0;
        for (kAcol = 0; kAcol < 6; kAcol++) {
          smax += K[i + (kAcol << 4)] * H[kAcol + 6 * jp1j];
        }
        A[i + (jp1j << 4)] = smax;
      }
      for (jp1j = 0; jp1j < 16; jp1j++) {
        smax = 0.0;
        for (kAcol = 0; kAcol < 16; kAcol++) {
          smax += A[i + (kAcol << 4)] * Pp[kAcol + (jp1j << 4)];
        }
        jA = i + (jp1j << 4);
        b_P[jA] = Pp[jA] - smax;
      }
    }
  } else {
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
  Posn->re = PosP[0];
  Posn->im = PosP[2];
  Headn[0] = Hroll;
  Headn[1] = Hpitch;
  Headn[2] = Hyaw;
  /*   */
  /*  totHead = [totHead [Hroll;Hpitch;Hyaw]]; */
  /*  totA = [totA [roll;pitch;yaw]]; */
  /*  totHUWB = [totHUWB HeadUWB]; */
  /*   */
  /*  totPosP = [totPosP PosP]; */
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
    H[i] = iv[i];
  }
  for (i = 0; i < 36; i++) {
    R[i] = iv1[i];
  }
  Hroll = 0.0;
  Hpitch = 0.0;
  Hyaw = 0.0;
  /*  s = 8.14; */
  PosP[0] = 0.0;
  PosP[1] = 0.0;
  PosP[2] = 0.0;
  for (i = 0; i < 256; i++) {
    Q[i] = 1.0E-5;
  }
  /*  Q = 1e-3*ones(16,16); */
  for (i = 0; i < 217; i++) {
    Q[uv[i]] = 0.0;
  }
  for (i = 0; i < 256; i++) {
    b_P[i] = 1.0;
  }
  for (i = 0; i < 217; i++) {
    b_P[uv[i]] = 0.0;
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
  s = 4.5839;
  roll = -0.006;
  pitch = -4.4647;
  yaw = -0.0458;
  /*  roll =  0; */
  /*  pitch = 0; */
  /*  yaw =  0; */
  /*  s = 4.6; */
  /*  sA = s*eulr2dcm([pi*0 -pi*0.57 pi*0]); */
  /*   */
  /*  SLAMposInit = [40.572;0;4.2114]; */
  /*  roll = 0; */
  /*  pitch = -pi*0.57; */
  /*  yaw = 0; */
  /*  TrMat = eulr2dcm([pi/2 0 pi/2]); */
  /*  Tempp = TrMat*sA/s; */
  /*  Tep = dcm2eulr(Tempp)-[pi/2; 0; pi/2]; */
  /*  roll = Tep(1); */
  /*  pitch = Tep(3); */
  /*  yaw = -Tep(2); */
}

void fx(const double xhat[16], double xp[16])
{
  double b_xn_tmp;
  double c_xn_tmp;
  double d_xn_tmp;
  double e_xn_tmp;
  double f_xn_tmp;
  double q;
  double r;
  double xn_tmp;
  /* ------------------------ */
  xn_tmp = cos(xhat[6]);
  b_xn_tmp = sin(xhat[6]);
  c_xn_tmp = sin(xhat[8]);
  d_xn_tmp = cos(xhat[8]);
  e_xn_tmp = cos(xhat[7]);
  f_xn_tmp = sin(xhat[7]);
  if (rtIsNaN(xhat[14] + 3.1415926535897931) ||
      rtIsInf(xhat[14] + 3.1415926535897931)) {
    r = rtNaN;
  } else if (xhat[14] + 3.1415926535897931 == 0.0) {
    r = 0.0;
  } else {
    boolean_T rEQ0;
    r = fmod(xhat[14] + 3.1415926535897931, 6.2831853071795862);
    rEQ0 = (r == 0.0);
    if (!rEQ0) {
      q = fabs((xhat[14] + 3.1415926535897931) / 6.2831853071795862);
      rEQ0 = !(fabs(q - floor(q + 0.5)) > 2.2204460492503131E-16 * q);
    }
    if (rEQ0) {
      r = 0.0;
    } else if (xhat[14] + 3.1415926535897931 < 0.0) {
      r += 6.2831853071795862;
    }
  }
  q = xn_tmp * f_xn_tmp;
  xp[0] = xhat[0] +
          xhat[9] *
              ((((xn_tmp * e_xn_tmp * xhat[3] - b_xn_tmp * d_xn_tmp * xhat[4]) +
                 q * c_xn_tmp * xhat[4]) +
                b_xn_tmp * c_xn_tmp * xhat[5]) +
               q * d_xn_tmp * xhat[5]);
  q = b_xn_tmp * f_xn_tmp;
  xp[1] = xhat[1] +
          xhat[9] *
              ((((b_xn_tmp * e_xn_tmp * xhat[3] + xn_tmp * d_xn_tmp * xhat[4]) +
                 q * c_xn_tmp * xhat[4]) -
                xn_tmp * c_xn_tmp * xhat[5]) +
               q * d_xn_tmp * xhat[5]);
  xp[2] = xhat[2] +
          xhat[9] * ((-f_xn_tmp * xhat[3] + e_xn_tmp * c_xn_tmp * xhat[4]) +
                     e_xn_tmp * d_xn_tmp * xhat[5]);
  xp[3] = xhat[3];
  xp[4] = xhat[4];
  xp[5] = xhat[5];
  xp[6] = xhat[6];
  xp[7] = xhat[7];
  xp[8] = xhat[8];
  xp[9] = xhat[9];
  xp[10] =
      (xhat[10] + b_mod(xhat[13] + 1.5707963267948966)) - 1.5707963267948966;
  xp[11] = (xhat[11] + r) - 3.1415926535897931;
  xp[12] =
      (xhat[12] + b_mod(xhat[15] + 1.5707963267948966)) - 1.5707963267948966;
  xp[13] = xhat[13];
  xp[14] = xhat[14];
  xp[15] = xhat[15];
}

/* End of code generation (EKF_UWB_SLAM_4.c) */
