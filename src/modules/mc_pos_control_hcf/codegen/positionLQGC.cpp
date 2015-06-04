/*
 * positionLQGC.cpp
 *
 * Code generation for function 'positionLQGC'
 *
 * C source code generated on: Mon Aug 18 14:58:54 2014
 *
 */

/* Remove floating point error. */
#pragma GCC diagnostic ignored "-Wfloat-equal"

/* Include files */
#include "rt_nonfinite.h"
#include "positionLQGC.h"

/* Variable Definitions */
static float P_apo_small[49];
static float z_E[10];

/* Function Declarations */
static void b_diag(const boolean_T v[3], boolean_T d[9]);
static void b_get_baro_C_line(float position_z, float local_altitude_offset,
  float C_line_baro[7]);
static void b_kron(const float A[4], const float B[9], float K[36]);
static void b_measurementUpdateSmall(const float x_apr[10], const float P_apr[49],
  const float C[7], float R, float y, float x_apo[10], float P_apo[49]);
static void c_diag(const float v[6], float d[36]);
static void c_kron(const double A[9], const boolean_T B[9], double K[81]);
static void c_measurementUpdateSmall(const float x_apr[10], const float P_apr[49],
  const float C[21], const float R[9], const float y[4], float x_apo[10], float
  P_apo[49]);
static void d_diag(const float v[3], float d[9]);
static void d_kron(const double A[4], const boolean_T B[9], double K[36]);
static void d_measurementUpdateSmall(const float x_apr[10], const float P_apr[49],
  const double C[14], const float R[4], const float y[3], float x_apo[10], float
  P_apo[49]);
static void diag(const float v[3], float d[9]);
static void e_diag(const float v[2], float d[4]);
static void e_kron(const double A[9], const double B[3], double K[27]);
static void f_kron(const double A[4], const double B[3], double K[12]);
static float getPressure(float h, float p_offset);
static void get_baro_C_line(float position_z, float local_altitude_offset, float
  C_line_baro[10]);
static void kron(const float A[9], const float B[9], float K[81]);
static void limit_u(const float u[3], const float uLim[3], float uOut[3]);
static void measurementUpdateSmall(const float x_apr[10], const float P_apr[49],
  const float C[49], const float R[49], const float y[10], float x_apo[10],
  float P_apo[49]);
static void mrdivide(const float A[49], const float B[49], float y[49]);
static void positionLQGC_init();
static float pressure_measurement_jacobian(float alt);
static float rt_powf_snf(float u0, float u1);

/* Function Definitions */
static void b_diag(const boolean_T v[3], boolean_T d[9])
{
  int j;
  for (j = 0; j < 9; j++) {
    d[j] = FALSE;
  }

  for (j = 0; j < 3; j++) {
    d[j + 3 * j] = v[j];
  }
}

static void b_get_baro_C_line(float position_z, float local_altitude_offset,
  float C_line_baro[7])
{
  int i8;

  /* helper function to construct the line in the measurement matrix C for the */
  /* baro measurement */
  /*  if small is set to true the function will return the line for the version */
  /*  where the x and y axis are handled together */
  /* last estimated altitude   */
  /*  returns the jacobian of the barometer measurement model */
  /*  see https://en.wikipedia.org/wiki/Barometric_formula */
  /*  assuming the aircraft is below 11000m */
  for (i8 = 0; i8 < 3; i8++) {
    C_line_baro[i8] = 0.0F;
  }

  C_line_baro[3] = -(28780.7305F * rt_powf_snf(288.15F / ((position_z +
    local_altitude_offset) * -0.0065F + 288.15F), -4.25587606F)) / 2395.77124F;
  C_line_baro[4] = 0.0F;
  C_line_baro[5] = 0.0F;
  C_line_baro[6] = 1.0F;
}

static void b_kron(const float A[4], const float B[9], float K[36])
{
  int kidx;
  int b_j1;
  int j2;
  int i1;
  int i2;
  kidx = -1;
  for (b_j1 = 0; b_j1 < 2; b_j1++) {
    for (j2 = 0; j2 < 3; j2++) {
      for (i1 = 0; i1 < 2; i1++) {
        for (i2 = 0; i2 < 3; i2++) {
          kidx++;
          K[kidx] = A[i1 + (b_j1 << 1)] * B[i2 + 3 * j2];
        }
      }
    }
  }
}

static void b_measurementUpdateSmall(const float x_apr[10], const float P_apr[49],
  const float C[7], float R, float y, float x_apo[10], float P_apo[49])
{
  float S;
  int i;
  float L[7];
  int i5;
  float f1;
  signed char I[49];
  float b_I[49];
  int i6;

  /*  Helper function to perform the kalman measurement update step while doing */
  /*  the covariance update only for x and y in oner step */
  /*  C is assumed to be already in teh correct (compact format) */
  /*  y: innovation */
  /*  mode: 0 = all sensors udpated, 1 = baro only, 2 = accel and baro, 3 = */
  /*  accelerometers only */
  S = 0.0F;
  for (i = 0; i < 7; i++) {
    L[i] = 0.0F;
    for (i5 = 0; i5 < 7; i5++) {
      L[i] += C[i5] * P_apr[i5 + 7 * i];
    }

    S += L[i] * C[i];
  }

  S += R;

  /*  innovation covariance */
  for (i = 0; i < 7; i++) {
    f1 = 0.0F;
    for (i5 = 0; i5 < 7; i5++) {
      f1 += P_apr[i + 7 * i5] * C[i5];
    }

    L[i] = f1 / S;
  }

  /*  kalman gain */
  for (i = 0; i < 49; i++) {
    I[i] = 0;
  }

  for (i = 0; i < 7; i++) {
    I[i + 7 * i] = 1;
  }

  for (i = 0; i < 7; i++) {
    for (i5 = 0; i5 < 7; i5++) {
      b_I[i + 7 * i5] = (float)I[i + 7 * i5] - L[i] * C[i5];
    }
  }

  for (i = 0; i < 7; i++) {
    for (i5 = 0; i5 < 7; i5++) {
      P_apo[i + 7 * i5] = 0.0F;
      for (i6 = 0; i6 < 7; i6++) {
        P_apo[i + 7 * i5] += b_I[i + 7 * i6] * P_apr[i6 + 7 * i5];
      }
    }
  }

  /*  calculate the _full_ state: x_apo = x_apr+L*y --> "un-merge" x and y axes, x */
  /*  and y axes values are calculated with the same values of L */
  for (i = 0; i < 10; i++) {
    x_apo[i] = 0.0F;
  }

  /* only one sensor */
  for (i = 0; i < 3; i++) {
    x_apo[i] = x_apr[i] + L[i] * y;
  }

  for (i = 0; i < 7; i++) {
    x_apo[i + 3] = x_apr[i + 3] + L[i] * y;
  }
}

static void c_diag(const float v[6], float d[36])
{
  int j;
  memset(&d[0], 0, 36U * sizeof(float));
  for (j = 0; j < 6; j++) {
    d[j + 6 * j] = v[j];
  }
}

static void c_kron(const double A[9], const boolean_T B[9], double K[81])
{
  int kidx;
  int b_j1;
  int j2;
  int i1;
  int i2;
  kidx = -1;
  for (b_j1 = 0; b_j1 < 3; b_j1++) {
    for (j2 = 0; j2 < 3; j2++) {
      for (i1 = 0; i1 < 3; i1++) {
        for (i2 = 0; i2 < 3; i2++) {
          kidx++;
          K[kidx] = A[i1 + 3 * b_j1] * (double)B[i2 + 3 * j2];
        }
      }
    }
  }
}

static void c_measurementUpdateSmall(const float x_apr[10], const float P_apr[49],
  const float C[21], const float R[9], const float y[4], float x_apo[10], float
  P_apo[49])
{
  float L[21];
  int rtemp;
  int k;
  int r1;
  float A[9];
  float maxval;
  float B[21];
  int r2;
  int r3;
  float a21;
  float Y[21];
  signed char I[49];
  static float b_I[49];
  float b_y[3];
  float b_x_apr[3];
  float c_x_apr[7];

  /*  Helper function to perform the kalman measurement update step while doing */
  /*  the covariance update only for x and y in oner step */
  /*  C is assumed to be already in teh correct (compact format) */
  /*  y: innovation */
  /*  mode: 0 = all sensors udpated, 1 = baro only, 2 = accel and baro, 3 = */
  /*  accelerometers only */
  /*  innovation covariance */
  for (rtemp = 0; rtemp < 3; rtemp++) {
    for (k = 0; k < 7; k++) {
      L[rtemp + 3 * k] = 0.0F;
      for (r1 = 0; r1 < 7; r1++) {
        L[rtemp + 3 * k] += C[rtemp + 3 * r1] * P_apr[r1 + 7 * k];
      }
    }
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    for (k = 0; k < 3; k++) {
      maxval = 0.0F;
      for (r1 = 0; r1 < 7; r1++) {
        maxval += L[k + 3 * r1] * C[rtemp + 3 * r1];
      }

      A[rtemp + 3 * k] = maxval + R[k + 3 * rtemp];
    }
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    for (k = 0; k < 7; k++) {
      B[rtemp + 3 * k] = 0.0F;
      for (r1 = 0; r1 < 7; r1++) {
        B[rtemp + 3 * k] += P_apr[k + 7 * r1] * C[rtemp + 3 * r1];
      }
    }
  }

  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = (real32_T)fabs(A[0]);
  a21 = (real32_T)fabs(A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if ((real32_T)fabs(A[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  A[r2] /= A[r1];
  A[r3] /= A[r1];
  A[3 + r2] -= A[r2] * A[3 + r1];
  A[3 + r3] -= A[r3] * A[3 + r1];
  A[6 + r2] -= A[r2] * A[6 + r1];
  A[6 + r3] -= A[r3] * A[6 + r1];
  if ((real32_T)fabs(A[3 + r3]) > (real32_T)fabs(A[3 + r2])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  A[3 + r3] /= A[3 + r2];
  A[6 + r3] -= A[3 + r3] * A[6 + r2];
  for (k = 0; k < 7; k++) {
    Y[3 * k] = B[r1 + 3 * k];
    Y[1 + 3 * k] = B[r2 + 3 * k] - Y[3 * k] * A[r2];
    Y[2 + 3 * k] = (B[r3 + 3 * k] - Y[3 * k] * A[r3]) - Y[1 + 3 * k] * A[3 + r3];
    Y[2 + 3 * k] /= A[6 + r3];
    Y[3 * k] -= Y[2 + 3 * k] * A[6 + r1];
    Y[1 + 3 * k] -= Y[2 + 3 * k] * A[6 + r2];
    Y[1 + 3 * k] /= A[3 + r2];
    Y[3 * k] -= Y[1 + 3 * k] * A[3 + r1];
    Y[3 * k] /= A[r1];
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    for (k = 0; k < 7; k++) {
      L[k + 7 * rtemp] = Y[rtemp + 3 * k];
    }
  }

  /*  kalman gain */
  for (rtemp = 0; rtemp < 49; rtemp++) {
    I[rtemp] = 0;
  }

  for (k = 0; k < 7; k++) {
    I[k + 7 * k] = 1;
  }

  for (rtemp = 0; rtemp < 7; rtemp++) {
    for (k = 0; k < 7; k++) {
      maxval = 0.0F;
      for (r1 = 0; r1 < 3; r1++) {
        maxval += L[rtemp + 7 * r1] * C[r1 + 3 * k];
      }

      b_I[rtemp + 7 * k] = (float)I[rtemp + 7 * k] - maxval;
    }
  }

  for (rtemp = 0; rtemp < 7; rtemp++) {
    for (k = 0; k < 7; k++) {
      P_apo[rtemp + 7 * k] = 0.0F;
      for (r1 = 0; r1 < 7; r1++) {
        P_apo[rtemp + 7 * k] += b_I[rtemp + 7 * r1] * P_apr[r1 + 7 * k];
      }
    }
  }

  /*  calculate the _full_ state: x_apo = x_apr+L*y --> "un-merge" x and y axes, x */
  /*  and y axes values are calculated with the same values of L */
  for (rtemp = 0; rtemp < 10; rtemp++) {
    x_apo[rtemp] = 0.0F;
  }

  /*  accelerometers and baro */
  b_y[0] = y[0];
  for (rtemp = 0; rtemp < 2; rtemp++) {
    b_y[rtemp + 1] = y[2 + rtemp];
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    maxval = 0.0F;
    for (k = 0; k < 3; k++) {
      maxval += L[rtemp + 7 * k] * b_y[k];
    }

    b_x_apr[rtemp] = x_apr[rtemp] + maxval;
  }

  for (rtemp = 0; rtemp < 3; rtemp++) {
    x_apo[rtemp] = b_x_apr[rtemp];
  }

  for (rtemp = 0; rtemp < 7; rtemp++) {
    maxval = 0.0F;
    for (k = 0; k < 3; k++) {
      maxval += L[rtemp + 7 * k] * y[1 + k];
    }

    c_x_apr[rtemp] = x_apr[3 + rtemp] + maxval;
  }

  for (rtemp = 0; rtemp < 7; rtemp++) {
    x_apo[3 + rtemp] = c_x_apr[rtemp];
  }
}

static void d_diag(const float v[3], float d[9])
{
  int j;
  for (j = 0; j < 9; j++) {
    d[j] = 0.0F;
  }

  for (j = 0; j < 3; j++) {
    d[j + 3 * j] = v[j];
  }
}

static void d_kron(const double A[4], const boolean_T B[9], double K[36])
{
  int kidx;
  int b_j1;
  int j2;
  int i1;
  int i2;
  kidx = -1;
  for (b_j1 = 0; b_j1 < 2; b_j1++) {
    for (j2 = 0; j2 < 3; j2++) {
      for (i1 = 0; i1 < 2; i1++) {
        for (i2 = 0; i2 < 3; i2++) {
          kidx++;
          K[kidx] = A[i1 + (b_j1 << 1)] * (double)B[i2 + 3 * j2];
        }
      }
    }
  }
}

static void d_measurementUpdateSmall(const float x_apr[10], const float P_apr[49],
  const double C[14], const float R[4], const float y[3], float x_apo[10], float
  P_apo[49])
{
  float L[14];
  int r1;
  int r2;
  int k;
  float A[4];
  float a21;
  float B[14];
  float a22;
  float Y[14];
  signed char I[49];
  static float b_I[49];
  float b_y[2];
  float b_x_apr[3];
  float c_x_apr[7];

  /*  Helper function to perform the kalman measurement update step while doing */
  /*  the covariance update only for x and y in oner step */
  /*  C is assumed to be already in teh correct (compact format) */
  /*  y: innovation */
  /*  mode: 0 = all sensors udpated, 1 = baro only, 2 = accel and baro, 3 = */
  /*  accelerometers only */
  /*  innovation covariance */
  for (r1 = 0; r1 < 2; r1++) {
    for (r2 = 0; r2 < 7; r2++) {
      L[r1 + (r2 << 1)] = 0.0F;
      for (k = 0; k < 7; k++) {
        L[r1 + (r2 << 1)] += (float)C[r1 + (k << 1)] * P_apr[k + 7 * r2];
      }
    }
  }

  for (r1 = 0; r1 < 2; r1++) {
    for (r2 = 0; r2 < 2; r2++) {
      a21 = 0.0F;
      for (k = 0; k < 7; k++) {
        a21 += L[r2 + (k << 1)] * (float)C[r1 + (k << 1)];
      }

      A[r1 + (r2 << 1)] = a21 + R[r2 + (r1 << 1)];
    }
  }

  for (r1 = 0; r1 < 2; r1++) {
    for (r2 = 0; r2 < 7; r2++) {
      B[r1 + (r2 << 1)] = 0.0F;
      for (k = 0; k < 7; k++) {
        B[r1 + (r2 << 1)] += P_apr[r2 + 7 * k] * (float)C[r1 + (k << 1)];
      }
    }
  }

  if ((real32_T)fabs(A[1]) > (real32_T)fabs(A[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }

  a21 = A[r2] / A[r1];
  a22 = A[2 + r2] - a21 * A[2 + r1];
  for (k = 0; k < 7; k++) {
    Y[1 + (k << 1)] = (B[r2 + (k << 1)] - B[r1 + (k << 1)] * a21) / a22;
    Y[k << 1] = (B[r1 + (k << 1)] - Y[1 + (k << 1)] * A[2 + r1]) / A[r1];
  }

  for (r1 = 0; r1 < 2; r1++) {
    for (r2 = 0; r2 < 7; r2++) {
      L[r2 + 7 * r1] = Y[r1 + (r2 << 1)];
    }
  }

  /*  kalman gain */
  for (r1 = 0; r1 < 49; r1++) {
    I[r1] = 0;
  }

  for (k = 0; k < 7; k++) {
    I[k + 7 * k] = 1;
  }

  for (r1 = 0; r1 < 7; r1++) {
    for (r2 = 0; r2 < 7; r2++) {
      a21 = 0.0F;
      for (k = 0; k < 2; k++) {
        a21 += L[r1 + 7 * k] * (float)C[k + (r2 << 1)];
      }

      b_I[r1 + 7 * r2] = (float)I[r1 + 7 * r2] - a21;
    }
  }

  for (r1 = 0; r1 < 7; r1++) {
    for (r2 = 0; r2 < 7; r2++) {
      P_apo[r1 + 7 * r2] = 0.0F;
      for (k = 0; k < 7; k++) {
        P_apo[r1 + 7 * r2] += b_I[r1 + 7 * k] * P_apr[k + 7 * r2];
      }
    }
  }

  /*  calculate the _full_ state: x_apo = x_apr+L*y --> "un-merge" x and y axes, x */
  /*  and y axes values are calculated with the same values of L */
  for (r1 = 0; r1 < 10; r1++) {
    x_apo[r1] = 0.0F;
  }

  /*  accelerometers only */
  b_y[0] = y[0];
  b_y[1] = y[2];
  for (r1 = 0; r1 < 3; r1++) {
    a21 = 0.0F;
    for (r2 = 0; r2 < 2; r2++) {
      a21 += L[r1 + 7 * r2] * b_y[r2];
    }

    b_x_apr[r1] = x_apr[r1] + a21;
  }

  for (r1 = 0; r1 < 3; r1++) {
    x_apo[r1] = b_x_apr[r1];
  }

  for (r1 = 0; r1 < 7; r1++) {
    a21 = 0.0F;
    for (r2 = 0; r2 < 2; r2++) {
      a21 += L[r1 + 7 * r2] * y[1 + r2];
    }

    c_x_apr[r1] = x_apr[3 + r1] + a21;
  }

  for (r1 = 0; r1 < 7; r1++) {
    x_apo[3 + r1] = c_x_apr[r1];
  }
}

static void diag(const float v[3], float d[9])
{
  int j;
  for (j = 0; j < 9; j++) {
    d[j] = 0.0F;
  }

  for (j = 0; j < 3; j++) {
    d[j + 3 * j] = v[j];
  }
}

static void e_diag(const float v[2], float d[4])
{
  int j;
  for (j = 0; j < 4; j++) {
    d[j] = 0.0F;
  }

  for (j = 0; j < 2; j++) {
    d[j + (j << 1)] = v[j];
  }
}

static void e_kron(const double A[9], const double B[3], double K[27])
{
  int kidx;
  int b_j1;
  int j2;
  int i1;
  kidx = -1;
  for (b_j1 = 0; b_j1 < 3; b_j1++) {
    for (j2 = 0; j2 < 3; j2++) {
      for (i1 = 0; i1 < 3; i1++) {
        kidx++;
        K[kidx] = A[i1 + 3 * b_j1] * B[j2];
      }
    }
  }
}

static void f_kron(const double A[4], const double B[3], double K[12])
{
  int kidx;
  int b_j1;
  int j2;
  int i1;
  kidx = -1;
  for (b_j1 = 0; b_j1 < 2; b_j1++) {
    for (j2 = 0; j2 < 3; j2++) {
      for (i1 = 0; i1 < 2; i1++) {
        kidx++;
        K[kidx] = A[i1 + (b_j1 << 1)] * B[j2];
      }
    }
  }
}

static float getPressure(float h, float p_offset)
{
  /* see https://en.wikipedia.org/wiki/Barometric_formula */
  return p_offset + 101325.0F * rt_powf_snf(288.15F / (288.15F + -0.0065F * h),
    -5.25587606F);
}

static void get_baro_C_line(float position_z, float local_altitude_offset, float
  C_line_baro[10])
{
  int i7;

  /* helper function to construct the line in the measurement matrix C for the */
  /* baro measurement */
  /*  if small is set to true the function will return the line for the version */
  /*  where the x and y axis are handled together */
  /* last estimated altitude   */
  /*  returns the jacobian of the barometer measurement model */
  /*  see https://en.wikipedia.org/wiki/Barometric_formula */
  /*  assuming the aircraft is below 11000m */
  for (i7 = 0; i7 < 6; i7++) {
    C_line_baro[i7] = 0.0F;
  }

  C_line_baro[6] = -(28780.7305F * rt_powf_snf(288.15F / ((position_z +
    local_altitude_offset) * -0.0065F + 288.15F), -4.25587606F)) / 2395.77124F;
  C_line_baro[7] = 0.0F;
  C_line_baro[8] = 0.0F;
  C_line_baro[9] = 1.0F;
}

static void kron(const float A[9], const float B[9], float K[81])
{
  int kidx;
  int b_j1;
  int j2;
  int i1;
  int i2;
  kidx = -1;
  for (b_j1 = 0; b_j1 < 3; b_j1++) {
    for (j2 = 0; j2 < 3; j2++) {
      for (i1 = 0; i1 < 3; i1++) {
        for (i2 = 0; i2 < 3; i2++) {
          kidx++;
          K[kidx] = A[i1 + 3 * b_j1] * B[i2 + 3 * j2];
        }
      }
    }
  }
}

static void limit_u(const float u[3], const float uLim[3], float uOut[3])
{
  int k;
  float extremum;

  /*  Limits each element of the nx1 vector u by the element in the nx1 vector */
  /*  uLim such that for each element u_i: */
  /*  -uLim_i <= u_i <= uLim_i */
  for (k = 0; k < 3; k++) {
    if ((u[k] <= uLim[k]) || rtIsNaNF(uLim[k])) {
      extremum = u[k];
    } else {
      extremum = uLim[k];
    }

    if ((extremum >= -uLim[k]) || rtIsNaNF(-uLim[k])) {
      uOut[k] = extremum;
    } else {
      uOut[k] = -uLim[k];
    }
  }
}

static void measurementUpdateSmall(const float x_apr[10], const float P_apr[49],
  const float C[49], const float R[49], const float y[10], float x_apo[10],
  float P_apo[49])
{
  static float b_P_apr[49];
  static float b_C[49];
  int i;
  int i3;
  int i4;
  static float c_C[49];
  float f0;
  static float L[49];
  signed char I[49];
  float b_y[7];
  float b_x_apr[3];

  /*  Helper function to perform the kalman measurement update step while doing */
  /*  the covariance update only for x and y in oner step */
  /*  C is assumed to be already in teh correct (compact format) */
  /*  y: innovation */
  /*  mode: 0 = all sensors udpated, 1 = baro only, 2 = accel and baro, 3 = */
  /*  accelerometers only */
  /*  innovation covariance */
  for (i = 0; i < 7; i++) {
    for (i3 = 0; i3 < 7; i3++) {
      b_P_apr[i + 7 * i3] = 0.0F;
      for (i4 = 0; i4 < 7; i4++) {
        b_P_apr[i + 7 * i3] += P_apr[i + 7 * i4] * C[i3 + 7 * i4];
      }

      b_C[i + 7 * i3] = 0.0F;
      for (i4 = 0; i4 < 7; i4++) {
        b_C[i + 7 * i3] += C[i + 7 * i4] * P_apr[i4 + 7 * i3];
      }
    }
  }

  for (i = 0; i < 7; i++) {
    for (i3 = 0; i3 < 7; i3++) {
      f0 = 0.0F;
      for (i4 = 0; i4 < 7; i4++) {
        f0 += b_C[i + 7 * i4] * C[i3 + 7 * i4];
      }

      c_C[i + 7 * i3] = f0 + R[i + 7 * i3];
    }
  }

  mrdivide(b_P_apr, c_C, L);

  /*  kalman gain */
  for (i = 0; i < 49; i++) {
    I[i] = 0;
  }

  for (i = 0; i < 7; i++) {
    I[i + 7 * i] = 1;
  }

  for (i = 0; i < 7; i++) {
    for (i3 = 0; i3 < 7; i3++) {
      f0 = 0.0F;
      for (i4 = 0; i4 < 7; i4++) {
        f0 += L[i + 7 * i4] * C[i4 + 7 * i3];
      }

      b_C[i + 7 * i3] = (float)I[i + 7 * i3] - f0;
    }
  }

  for (i = 0; i < 7; i++) {
    for (i3 = 0; i3 < 7; i3++) {
      P_apo[i + 7 * i3] = 0.0F;
      for (i4 = 0; i4 < 7; i4++) {
        P_apo[i + 7 * i3] += b_C[i + 7 * i4] * P_apr[i4 + 7 * i3];
      }
    }
  }

  /*  calculate the _full_ state: x_apo = x_apr+L*y --> "un-merge" x and y axes, x */
  /*  and y axes values are calculated with the same values of L */
  for (i = 0; i < 10; i++) {
    x_apo[i] = 0.0F;
  }

  /*  full update */
  for (i = 0; i < 3; i++) {
    b_y[i] = y[i];
  }

  for (i = 0; i < 4; i++) {
    b_y[i + 3] = y[6 + i];
  }

  for (i = 0; i < 3; i++) {
    f0 = 0.0F;
    for (i3 = 0; i3 < 7; i3++) {
      f0 += L[i + 7 * i3] * b_y[i3];
    }

    b_x_apr[i] = x_apr[i] + f0;
  }

  for (i = 0; i < 3; i++) {
    x_apo[i] = b_x_apr[i];
  }

  for (i = 0; i < 7; i++) {
    f0 = 0.0F;
    for (i3 = 0; i3 < 7; i3++) {
      f0 += L[i + 7 * i3] * y[3 + i3];
    }

    b_y[i] = x_apr[3 + i] + f0;
  }

  for (i = 0; i < 7; i++) {
    x_apo[3 + i] = b_y[i];
  }
}

static void mrdivide(const float A[49], const float B[49], float y[49])
{
  static float b_A[49];
  signed char ipiv[7];
  int i2;
  int iy;
  int j;
  int c;
  int ix;
  float temp;
  int k;
  float s;
  int jy;
  int ijA;
  static float Y[49];
  for (i2 = 0; i2 < 7; i2++) {
    for (iy = 0; iy < 7; iy++) {
      b_A[iy + 7 * i2] = B[i2 + 7 * iy];
    }

    ipiv[i2] = (signed char)(1 + i2);
  }

  for (j = 0; j < 6; j++) {
    c = j << 3;
    iy = 0;
    ix = c;
    temp = (real32_T)fabs(b_A[c]);
    for (k = 2; k <= 7 - j; k++) {
      ix++;
      s = (real32_T)fabs(b_A[ix]);
      if (s > temp) {
        iy = k - 1;
        temp = s;
      }
    }

    if (b_A[c + iy] != 0.0F) {
      if (iy != 0) {
        ipiv[j] = (signed char)((j + iy) + 1);
        ix = j;
        iy += j;
        for (k = 0; k < 7; k++) {
          temp = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = temp;
          ix += 7;
          iy += 7;
        }
      }

      i2 = (c - j) + 7;
      for (jy = c + 1; jy + 1 <= i2; jy++) {
        b_A[jy] /= b_A[c];
      }
    }

    iy = c;
    jy = c + 7;
    for (k = 1; k <= 6 - j; k++) {
      temp = b_A[jy];
      if (b_A[jy] != 0.0F) {
        ix = c + 1;
        i2 = (iy - j) + 14;
        for (ijA = 8 + iy; ijA + 1 <= i2; ijA++) {
          b_A[ijA] += b_A[ix] * -temp;
          ix++;
        }
      }

      jy += 7;
      iy += 7;
    }
  }

  for (i2 = 0; i2 < 7; i2++) {
    for (iy = 0; iy < 7; iy++) {
      Y[iy + 7 * i2] = A[i2 + 7 * iy];
    }
  }

  for (jy = 0; jy < 7; jy++) {
    if (ipiv[jy] != jy + 1) {
      for (j = 0; j < 7; j++) {
        temp = Y[jy + 7 * j];
        Y[jy + 7 * j] = Y[(ipiv[jy] + 7 * j) - 1];
        Y[(ipiv[jy] + 7 * j) - 1] = temp;
      }
    }
  }

  for (j = 0; j < 7; j++) {
    c = 7 * j;
    for (k = 0; k < 7; k++) {
      iy = 7 * k;
      if (Y[k + c] != 0.0F) {
        for (jy = k + 2; jy < 8; jy++) {
          Y[(jy + c) - 1] -= Y[k + c] * b_A[(jy + iy) - 1];
        }
      }
    }
  }

  for (j = 0; j < 7; j++) {
    c = 7 * j;
    for (k = 6; k > -1; k += -1) {
      iy = 7 * k;
      if (Y[k + c] != 0.0F) {
        Y[k + c] /= b_A[k + iy];
        for (jy = 0; jy + 1 <= k; jy++) {
          Y[jy + c] -= Y[k + c] * b_A[jy + iy];
        }
      }
    }
  }

  for (i2 = 0; i2 < 7; i2++) {
    for (iy = 0; iy < 7; iy++) {
      y[iy + 7 * i2] = Y[i2 + 7 * iy];
    }
  }
}

static void positionLQGC_init()
{
  static const float fv6[49] = { 10.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 100000.0F };

  int i;
  memcpy(&P_apo_small[0], &fv6[0], 49U * sizeof(float));
  for (i = 0; i < 10; i++) {
    z_E[i] = 0.0F;
  }
}

static float pressure_measurement_jacobian(float alt)
{
  /*  returns the jacobian of the barometer measurement model */
  /*  see https://en.wikipedia.org/wiki/Barometric_formula */
  /*  assuming the aircraft is below 11000m */
  return -(28780.7305F * rt_powf_snf(288.15F / (alt * -0.0065F + 288.15F),
            -4.25587606F)) / 2395.77124F;
}

static float rt_powf_snf(float u0, float u1)
{
  float y;
  float f2;
  float f3;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = ((real32_T)rtNaN);
  } else {
    f2 = (real32_T)fabs(u0);
    f3 = (real32_T)fabs(u1);
    if (rtIsInfF(u1)) {
      if (f2 == 1.0F) {
        y = ((real32_T)rtNaN);
      } else if (f2 > 1.0F) {
        if (u1 > 0.0F) {
          y = ((real32_T)rtInf);
        } else {
          y = 0.0F;
        }
      } else if (u1 > 0.0F) {
        y = 0.0F;
      } else {
        y = ((real32_T)rtInf);
      }
    } else if (f3 == 0.0F) {
      y = 1.0F;
    } else if (f3 == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
      y = (real32_T)sqrt(u0);
    } else if ((u0 < 0.0F) && (u1 > (real32_T)floor(u1))) {
      y = ((real32_T)rtNaN);
    } else {
      y = (real32_T)pow(u0, u1);
    }
  }

  return y;
}

void positionLQGC(float q, const float r_position[3], const float r_velocity[3],
                  const float r_acceleration[3], const float k[6], const float
                  x_apo_k[10], const float z[10], const boolean_T zFlag[4],
                  const float F_global[3], boolean_T intfreeze, const float
                  setpPosRaw[3], const float setpVelRaw[3], const float
                  setpAccRaw[3], boolean_T intReset, float, const boolean_T
                  useControlOutputForPrediction[2], float mass, float
                  q_baro_offset, float r_baro, float deltaT, float
                  local_altitude_offset, const float intLim[3], const float
                  uLim[3], float x_apo[10], float u[3], float F_desProp[3],
                  float *variance_horizontal, float *variance_vertical, float
                  debugOutput[4])
{
  int i;
  float setp[10];
  float z_B[10];
  static float K[30];
  float ie[3];
  float Kint[9];
  float dt;
  float st;
  float As[9];
  static const signed char iv0[3] = { 0, 0, 1 };

  static float A_nobaro[81];
  static const float fv0[9] = { 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F,
    1.0F };

  static float A_nobaro_small[36];
  static const float fv1[4] = { 1.0F, 0.0F, 0.0F, 1.0F };

  int i0;
  static float A[100];
  static const signed char iv1[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  static float A_small[49];
  static const signed char iv2[7] = { 0, 0, 0, 0, 0, 0, 1 };

  static float B[30];
  float b_K[3];
  float b_x_apo_k[10];
  float u_pre[3];
  float b_A[10];
  static float b_A_small[49];
  static float c_A_small[49];
  static float b_A_nobaro_small[49];
  int i1;
  static float P_apr_small[49];
  boolean_T b_zFlag[3];
  boolean_T bv0[9];
  static double dv0[81];
  static const double dv1[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

  boolean_T c_zFlag[3];
  boolean_T bv1[9];
  static double dv2[36];
  static const double dv3[4] = { 1.0, 0.0, 0.0, 1.0 };

  float b_r_position[6];
  static float R_nobaro_small[36];
  float fv2[7];
  static double dv4[27];
  static const double dv5[3] = { 0.0, 0.0, 1.0 };

  static float C[40];
  static double dv6[12];
  float C_small[21];
  float b_r_acceleration[3];
  float b_C[4];
  float fv3[4];
  static const signed char iv3[4] = { 2, 5, 8, 9 };

  float fv4[9];
  float c_r_acceleration[2];
  static const signed char iv4[30] = { 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 };

  float fv5[4];
  static const double dv7[14] = { 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0 };

  float F_desInt[3];
  float ff[3];

  /* LQG Postion Estimator and Controller */
  /*  Observer: */
  /*         x[n|n]   = x[n|n-1] + M(y[n] - Cx[n|n-1] - Du[n]) */
  /*         x[n+1|n] = Ax[n|n] + Bu[n] */
  /*  */
  /*  Tobias Naegeli */
  /*  Thomas Gubler  */
  /*  */
  /*  Arguments: */
  /*  q: process variance */
  /*  r_position: measurement variance */
  /*  r_velocity: velocity measurement variance */
  /*  k: control gains, 6x1 vector [k1xy;k2xy;k3xy;k1z;k2z;k3z] */
  /*  x_apo_k: state estimate of last timestep */
  /*  z: measurements vector of measurements [pos x, vel x, acc x, pos y, vel y, acc y, pos z, vel z, acc z, pressure] */
  /*  zFlag: vector with boolean values which indicates if a measurement from */
  /*  the sensor was received = [position; velocity; acceleration; barometer] */
  /*  F_global: 3d force vector which gets added to the output control force */
  /*  intfreeze: is set to true the controller will not integrate */
  /*  ie: integrator values */
  /*  setpPosRaw: position setpoint */
  /*  setpVelRaw: velocity setpoint */
  /*  setpAccRaw: acceleration setpoint */
  /*  intReset: reset intgerators */
  /*  z_E_last: z_E from last iteration */
  /*  dxmax: thershold for spice rejection */
  /*  useControlOutputForPrediction: 2x1 vector of flags [horizontal, vertical] */
  /*  if horizontal is true the control output is used to predict in x/y */
  /*  direction */
  /*  if vertical is true the control output is used to predict in z direction */
  /*  in the prediction step */
  /*  mass: mass of the system in kg */
  /*  q_baro_offset: process variance for barometer pressure offset */
  /*  r_baro: barometer measurement variance */
  /*  deltaT: time difference since last update */
  /*  local_altitude_offset: altitude difference between global altitude and the local */
  /*  frame */
  /*  intLim: 3x1 vector of control integral */
  /*  uLim: 3x1 vector limiting the control signal output (force) [ux_max; uy_max; uz_max] */
  /*  Output: */
  /*  ie_out: integrator values */
  /*  x_apo: state estimate */
  /*  u: contorl output */
  /*  F_desProp: */
  /*  P_apo_out: covariance estimate */
  /*  z_E; measurement after spice rejection */
  /*  debugOutput: vector with debug outputs */
  /* % configuration */
  /* % Index constants */
  /* States */
  /*  x position  (Index State Position X) */
  /*  x velocity  (Index State Velocity X) */
  /*  x acceleration  (Index State Acceleration X) */
  /*  y position */
  /*  y velocity */
  /*  y acceleration */
  /*  z position */
  /*  z velocity */
  /*  z acceleration */
  /*  barometer pressure offset */
  /* Measurements */
  /*  x position (Index Measurement Position X) */
  /*  x velocity */
  /*  x acceleration */
  /*  y position */
  /*  y velocity */
  /*  y acceleration */
  /*  z position */
  /*  z velocity */
  /*  z acceleration */
  /*  pressure */
  /* % init */
  for (i = 0; i < 4; i++) {
    debugOutput[i] = 0.0F;
  }

  /* % copying arguments */
  /*  Setpoint generation for a trajectory flight */
  /* setpoint for position */
  for (i = 0; i < 10; i++) {
    setp[i] = 0.0F;

    /* R_B=R_A; */
    /* % align the measurements into the swarm frame */
    /* align the measurements with the */
    z_B[i] = z[i];
  }

  setp[0] = setpPosRaw[0];

  /* x-setp */
  setp[3] = setpPosRaw[1];

  /* y-setp */
  setp[6] = setpPosRaw[2];

  /* z-setp */
  /* setpoint for velocity */
  setp[1] = setpVelRaw[0];

  /* vx-setp */
  setp[4] = setpVelRaw[1];

  /* vy-setp */
  setp[7] = setpVelRaw[2];

  /* vz-setp */
  /* setpoint for acceleration */
  setp[2] = setpAccRaw[0];
  setp[5] = setpAccRaw[1];
  setp[8] = setpAccRaw[2];

  /* feedforward for acceleration */
  z_B[6] = -z[6];

  /*  estimator uses z up! */
  z_B[7] = -z[7];

  /*  estimator uses z up! */
  z_B[8] = -z[8];

  /*  estimator uses z up! */
  /* z_E_raw=R_B'*z_B; %Transform vision measurements from the body into the inertial frame */
  /* % make a basic spice rejection */
  /* XXX extend for all measurements */
  for (i = 0; i < 10; i++) {
    z_E[i] = z_B[i];
  }

  /* % define the controlling gains */
  K[0] = k[0];
  K[3] = k[1];
  K[6] = 0.0F;
  K[9] = 0.0F;
  K[12] = 0.0F;
  K[15] = 0.0F;
  K[18] = 0.0F;
  K[21] = 0.0F;
  K[24] = 0.0F;
  K[1] = 0.0F;
  K[4] = 0.0F;
  K[7] = 0.0F;
  K[10] = k[0];
  K[13] = k[1];
  K[16] = 0.0F;
  K[19] = 0.0F;
  K[22] = 0.0F;
  K[25] = 0.0F;
  K[2] = 0.0F;
  K[5] = 0.0F;
  K[8] = 0.0F;
  K[11] = 0.0F;
  K[14] = 0.0F;
  K[17] = 0.0F;
  K[20] = k[3];
  K[23] = k[4];
  K[26] = 0.0F;
  ie[0] = k[2];
  ie[1] = k[2];
  ie[2] = k[5];
  diag(ie, Kint);

  /* % define the model */
  dt = deltaT / mass;
  st = 0.5F * (dt * dt) / mass;

  /* A: */
  As[0] = 1.0F;
  As[3] = dt;
  As[6] = st;
  As[1] = 0.0F;
  As[4] = 1.0F;
  As[7] = dt;
  for (i = 0; i < 3; i++) {
    K[27 + i] = 0.0F;
    As[2 + 3 * i] = iv0[i];
  }

  kron(fv0, As, A_nobaro);
  b_kron(fv1, As, A_nobaro_small);

  /* x and y axis together */
  /*  add barometer offset state */
  for (i = 0; i < 9; i++) {
    for (i0 = 0; i0 < 9; i0++) {
      A[i0 + 10 * i] = A_nobaro[i0 + 9 * i];
    }
  }

  for (i = 0; i < 9; i++) {
    A[90 + i] = 0.0F;
  }

  for (i = 0; i < 10; i++) {
    A[9 + 10 * i] = iv1[i];
  }

  for (i = 0; i < 6; i++) {
    for (i0 = 0; i0 < 6; i0++) {
      A_small[i0 + 7 * i] = A_nobaro_small[i0 + 6 * i];
    }
  }

  for (i = 0; i < 6; i++) {
    A_small[42 + i] = 0.0F;
  }

  for (i = 0; i < 7; i++) {
    A_small[6 + 7 * i] = iv2[i];
  }

  /* B: */
  /*  add barometer offset state */
  B[0] = st;
  B[10] = 0.0F;
  B[20] = 0.0F;
  B[1] = dt;
  B[11] = 0.0F;
  B[21] = 0.0F;
  for (i = 0; i < 3; i++) {
    B[2 + 10 * i] = 0.0F;
  }

  B[3] = 0.0F;
  B[13] = st;
  B[23] = 0.0F;
  B[4] = 0.0F;
  B[14] = dt;
  B[24] = 0.0F;
  for (i = 0; i < 3; i++) {
    B[5 + 10 * i] = 0.0F;
  }

  B[6] = 0.0F;
  B[16] = 0.0F;
  B[26] = st;
  B[7] = 0.0F;
  B[17] = 0.0F;
  B[27] = dt;

  /* Q: */
  /*  Q_nobaro  = kron(single(eye(3)),Qs); */
  for (i = 0; i < 3; i++) {
    B[8 + 10 * i] = 0.0F;
    B[9 + 10 * i] = 0.0F;
    As[3 * i] = 0.0F;
    As[1 + 3 * i] = 0.0F;
  }

  As[2] = 0.0F;
  As[5] = 0.0F;
  As[8] = q;
  b_kron(fv1, As, A_nobaro_small);

  /* x and y axis together */
  /*  add barometer offset process noise */
  /* Q = [Q_nobaro, zeros(NSTATES-1,1); [zeros(1, NSTATES-1), q_baro_offset]]; */
  /* % estimator */
  /*  prediction */
  for (i = 0; i < 10; i++) {
    b_x_apo_k[i] = x_apo_k[i] - setp[i];
  }

  for (i = 0; i < 3; i++) {
    b_K[i] = 0.0F;
    for (i0 = 0; i0 < 10; i0++) {
      b_K[i] += K[i + 3 * i0] * b_x_apo_k[i0];
    }
  }

  limit_u(b_K, uLim, u_pre);

  /*  applying the control output limit */
  if (useControlOutputForPrediction[0] && useControlOutputForPrediction[1]) {
    /*  full predicition with using control output in x,y,z directions */
    for (i = 0; i < 10; i++) {
      b_A[i] = 0.0F;
      for (i0 = 0; i0 < 10; i0++) {
        b_A[i] += A[i + 10 * i0] * x_apo_k[i0];
      }

      b_x_apo_k[i] = 0.0F;
      for (i0 = 0; i0 < 3; i0++) {
        b_x_apo_k[i] += B[i + 10 * i0] * u_pre[i0];
      }

      x_apo[i] = b_A[i] - b_x_apo_k[i];
    }
  } else if (useControlOutputForPrediction[0] &&
             (!useControlOutputForPrediction[1])) {
    /*  predicition with using control output in x, y direction */
    for (i = 0; i < 10; i++) {
      b_A[i] = 0.0F;
      for (i0 = 0; i0 < 10; i0++) {
        b_A[i] += A[i + 10 * i0] * x_apo_k[i0];
      }

      b_x_apo_k[i] = 0.0F;
      for (i0 = 0; i0 < 2; i0++) {
        b_x_apo_k[i] += B[i + 10 * i0] * u_pre[i0];
      }

      x_apo[i] = b_A[i] - b_x_apo_k[i];
    }
  } else if ((!useControlOutputForPrediction[0]) &&
             useControlOutputForPrediction[1]) {
    /*  predicition with using control output in z direction */
    for (i = 0; i < 10; i++) {
      st = 0.0F;
      for (i0 = 0; i0 < 10; i0++) {
        st += A[i + 10 * i0] * x_apo_k[i0];
      }

      x_apo[i] = st - B[20 + i] * u_pre[2];
    }
  } else {
    /*  predicition without using control output */
    for (i = 0; i < 10; i++) {
      x_apo[i] = 0.0F;
      for (i0 = 0; i0 < 10; i0++) {
        x_apo[i] += A[i + 10 * i0] * x_apo_k[i0];
      }
    }
  }

  /*  P_apr=A*(P_apo)*A'+Q; */
  for (i = 0; i < 7; i++) {
    for (i0 = 0; i0 < 7; i0++) {
      b_A_small[i + 7 * i0] = 0.0F;
      for (i1 = 0; i1 < 7; i1++) {
        b_A_small[i + 7 * i0] += A_small[i + 7 * i1] * P_apo_small[i1 + 7 * i0];
      }
    }

    for (i0 = 0; i0 < 7; i0++) {
      c_A_small[i + 7 * i0] = 0.0F;
      for (i1 = 0; i1 < 7; i1++) {
        c_A_small[i + 7 * i0] += b_A_small[i + 7 * i1] * A_small[i0 + 7 * i1];
      }
    }
  }

  for (i = 0; i < 6; i++) {
    for (i0 = 0; i0 < 6; i0++) {
      b_A_nobaro_small[i0 + 7 * i] = A_nobaro_small[i0 + 6 * i];
    }
  }

  for (i = 0; i < 6; i++) {
    b_A_nobaro_small[42 + i] = 0.0F;
  }

  for (i = 0; i < 6; i++) {
    b_A_nobaro_small[6 + 7 * i] = 0.0F;
  }

  b_A_nobaro_small[48] = q_baro_offset;
  for (i = 0; i < 7; i++) {
    for (i0 = 0; i0 < 7; i0++) {
      P_apr_small[i0 + 7 * i] = c_A_small[i0 + 7 * i] + b_A_nobaro_small[i0 + 7 *
        i];
    }
  }

  /*  P_apo = P_apr; */
  memcpy(&P_apo_small[0], &P_apr_small[0], 49U * sizeof(float));

  /* % update step */
  /*  filter out update steps where only the barometer was updated and then use */
  /*  a smaller measurement matrix */
  if (zFlag[0] || zFlag[1]) {
    /*  true if position or velocity measurement is available */
    /*  */
    /*  Update step with position OR velocity measurement and possibly barometer */
    /*  measurement */
    /*  */
    /* Define measurement matrix C, write a 1 into the matrix for the */
    /* measurements which updated */
    b_zFlag[0] = zFlag[0];
    b_zFlag[1] = zFlag[1];
    b_zFlag[2] = zFlag[2];
    b_diag(b_zFlag, bv0);
    c_kron(dv1, bv0, dv0);
    for (i = 0; i < 81; i++) {
      A_nobaro[i] = (float)dv0[i];
    }

    c_zFlag[0] = zFlag[0];
    c_zFlag[1] = zFlag[1];
    c_zFlag[2] = zFlag[2];
    b_diag(c_zFlag, bv1);
    d_kron(dv3, bv1, dv2);
    for (i = 0; i < 36; i++) {
      A_nobaro_small[i] = (float)dv2[i];
    }

    /* x and y axis together */
    /* Define measurement variance R: */
    b_r_position[0] = r_position[0];
    b_r_position[1] = r_velocity[0];
    b_r_position[2] = r_acceleration[0];
    b_r_position[3] = r_position[2];
    b_r_position[4] = r_velocity[2];
    b_r_position[5] = r_acceleration[2];
    c_diag(b_r_position, R_nobaro_small);

    /*  Check if baro (pressure) measurement is also available and expand R */
    /*  and C matrix with baro measurement */
    if (zFlag[3] == 1) {
      /*  true if set via zFlag argument (measurement available) */
      /* C: add barometer measurement: */
      /* see https://en.wikipedia.org/wiki/Barometric_formula */
      /* last estimated altitude             */
      st = pressure_measurement_jacobian(x_apo[6] + local_altitude_offset);

      /*  barometer measures pressure due to altitude + pressure offset */
      for (i = 0; i < 9; i++) {
        for (i0 = 0; i0 < 9; i0++) {
          A[i0 + 10 * i] = A_nobaro[i0 + 9 * i];
        }
      }

      for (i = 0; i < 9; i++) {
        A[90 + i] = 0.0F;
      }

      for (i = 0; i < 6; i++) {
        A[9 + 10 * i] = 0.0F;
        for (i0 = 0; i0 < 6; i0++) {
          A_small[i0 + 7 * i] = A_nobaro_small[i0 + 6 * i];
        }
      }

      A[69] = st;
      A[79] = 0.0F;
      A[89] = 0.0F;
      A[99] = 1.0F;
      for (i = 0; i < 6; i++) {
        A_small[42 + i] = 0.0F;
      }

      for (i = 0; i < 3; i++) {
        A_small[6 + 7 * i] = 0.0F;
      }

      A_small[27] = st;
      A_small[34] = 0.0F;
      A_small[41] = 0.0F;
      A_small[48] = 1.0F;
    } else {
      for (i = 0; i < 9; i++) {
        for (i0 = 0; i0 < 9; i0++) {
          A[i0 + 10 * i] = A_nobaro[i0 + 9 * i];
        }
      }

      for (i = 0; i < 9; i++) {
        A[90 + i] = 0.0F;
      }

      for (i = 0; i < 10; i++) {
        A[9 + 10 * i] = 0.0F;
      }

      for (i = 0; i < 6; i++) {
        for (i0 = 0; i0 < 6; i0++) {
          A_small[i0 + 7 * i] = A_nobaro_small[i0 + 6 * i];
        }
      }

      for (i = 0; i < 6; i++) {
        A_small[42 + i] = 0.0F;
      }

      for (i = 0; i < 7; i++) {
        A_small[6 + 7 * i] = 0.0F;
      }
    }

    /*  evaluate non-linear measurement model for the barometer */
    st = getPressure(x_apo[6] + local_altitude_offset, x_apo[9]);
    debugOutput[0] = st;

    /*  calculate estimated measurement: */
    for (i = 0; i < 9; i++) {
      As[i] = 0.0F;
      for (i0 = 0; i0 < 10; i0++) {
        As[i] += A[i + 10 * i0] * x_apo[i0];
      }

      b_A[i] = As[i];
    }

    b_A[9] = st;
    for (i = 0; i < 10; i++) {
      z_B[i] = z_E[i] - b_A[i];
    }

    /* innovation */
    /*  set innovation to 0 for measurements which did not update in this */
    /*  iteration */
    if (!zFlag[0]) {
      z_B[0] = 0.0F;
      z_B[3] = 0.0F;
      z_B[6] = 0.0F;
    }

    if (!zFlag[1]) {
      z_B[1] = 0.0F;
      z_B[4] = 0.0F;
      z_B[7] = 0.0F;
    }

    if (!zFlag[2]) {
      z_B[2] = 0.0F;
      z_B[5] = 0.0F;
      z_B[8] = 0.0F;
    }

    if (!zFlag[3]) {
      z_B[9] = 0.0F;
    }

    /*  calculate new covariance and state estimation: */
    for (i = 0; i < 6; i++) {
      for (i0 = 0; i0 < 6; i0++) {
        b_A_small[i0 + 7 * i] = R_nobaro_small[i0 + 6 * i];
      }
    }

    for (i = 0; i < 6; i++) {
      b_A_small[42 + i] = 0.0F;
    }

    for (i = 0; i < 6; i++) {
      b_A_small[6 + 7 * i] = 0.0F;
    }

    b_A_small[48] = r_baro;
    for (i = 0; i < 10; i++) {
      b_x_apo_k[i] = x_apo[i];
    }

    measurementUpdateSmall(b_x_apo_k, P_apr_small, A_small, b_A_small, z_B,
      x_apo, P_apo_small);
  } else if ((!zFlag[0]) && (!zFlag[1]) && (!zFlag[2]) && zFlag[3]) {
    /*  position/velocity/accleration measurement not available, baro measurement is available */
    /*  */
    /*  Update step with barometer as the only sensor */
    /*  */
    /* last estimated altitude             */
    /*  barometer measures pressure due to altitude + pressure offset */
    /*      C_baro = single([zeros(1, IM_PZ-1), dhdx_pressure, 0, 0, dhdx_pressureoffset]); */
    st = getPressure(x_apo[6] + local_altitude_offset, x_apo[9]);
    debugOutput[0] = st;

    /* innovation */
    /*  calculate new covariance and state estimation: */
    for (i = 0; i < 3; i++) {
      fv2[i] = 0.0F;
    }

    fv2[3] = pressure_measurement_jacobian(x_apo[6] + local_altitude_offset);
    fv2[4] = 0.0F;
    fv2[5] = 0.0F;
    fv2[6] = 1.0F;
    for (i = 0; i < 10; i++) {
      b_x_apo_k[i] = x_apo[i];
    }

    b_measurementUpdateSmall(b_x_apo_k, P_apr_small, fv2, r_baro, z_E[9] - st,
      x_apo, P_apo_small);
  } else if ((!zFlag[0]) && (!zFlag[1]) && zFlag[2] && zFlag[3]) {
    /*  position/velocity measurement not available, acceleration and baro measurement is available */
    e_kron(dv1, dv5, dv4);
    get_baro_C_line(x_apo[6], local_altitude_offset, b_x_apo_k);
    for (i = 0; i < 9; i++) {
      for (i0 = 0; i0 < 3; i0++) {
        C[i0 + (i << 2)] = (float)dv4[i0 + 3 * i];
      }
    }

    for (i = 0; i < 3; i++) {
      C[36 + i] = 0.0F;
    }

    for (i = 0; i < 10; i++) {
      C[3 + (i << 2)] = b_x_apo_k[i];
    }

    f_kron(dv3, dv5, dv6);
    b_get_baro_C_line(x_apo[6], local_altitude_offset, fv2);
    for (i = 0; i < 6; i++) {
      for (i0 = 0; i0 < 2; i0++) {
        C_small[i0 + 3 * i] = (float)dv6[i0 + (i << 1)];
      }
    }

    for (i = 0; i < 2; i++) {
      C_small[18 + i] = 0.0F;
    }

    for (i = 0; i < 7; i++) {
      C_small[2 + 3 * i] = fv2[i];
    }

    /*      R = single(diag([r_acceleration; r_baro])); */
    b_r_acceleration[0] = r_acceleration[0];
    b_r_acceleration[1] = r_acceleration[2];
    b_r_acceleration[2] = r_baro;
    st = getPressure(x_apo[6] + local_altitude_offset, x_apo[9]);
    debugOutput[0] = st;

    /*  calculate estimated measurement: */
    /* calculate innovation */
    /*  calculate new covariance and state estimation: */
    for (i = 0; i < 3; i++) {
      ie[i] = 0.0F;
      for (i0 = 0; i0 < 10; i0++) {
        ie[i] += C[i + (i0 << 2)] * x_apo[i0];
      }

      b_C[i] = ie[i];
    }

    b_C[3] = st;
    for (i = 0; i < 4; i++) {
      fv3[i] = z_E[iv3[i]] - b_C[i];
    }

    for (i = 0; i < 10; i++) {
      b_x_apo_k[i] = x_apo[i];
    }

    d_diag(b_r_acceleration, fv4);
    c_measurementUpdateSmall(b_x_apo_k, P_apr_small, C_small, fv4, fv3, x_apo,
      P_apo_small);
  } else {
    if ((!zFlag[0]) && (!zFlag[1]) && zFlag[2] && (!zFlag[3])) {
      /*  position/velocity and baro measurement not available, acceleration measurement is available */
      c_r_acceleration[0] = r_acceleration[0];
      c_r_acceleration[1] = r_acceleration[2];

      /*  calculate estimated measurement: */
      /* calculate innovation */
      /*  calculate new covariance and state estimation: */
      for (i = 0; i < 3; i++) {
        st = 0.0F;
        for (i0 = 0; i0 < 10; i0++) {
          st += (float)iv4[i + 3 * i0] * x_apo[i0];
        }

        ie[i] = z_E[2 + 3 * i] - st;
      }

      for (i = 0; i < 10; i++) {
        b_x_apo_k[i] = x_apo[i];
      }

      e_diag(c_r_acceleration, fv5);
      d_measurementUpdateSmall(b_x_apo_k, P_apr_small, dv7, fv5, ie, x_apo,
        P_apo_small);
    }
  }

  /*  variance_horizontal = max([P_apo(IS_PX, IS_PX), P_apo(IS_PY, IS_PY)]); */
  *variance_horizontal = P_apo_small[0];
  *variance_vertical = P_apo_small[24];

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /* % controlling part */
  for (i = 0; i < 10; i++) {
    setp[i] = x_apo[i] - setp[i];
  }

  u_pre[0] = setp[0];
  u_pre[1] = setp[2];
  u_pre[2] = setp[4];
  for (i = 0; i < 3; i++) {
    ie[i] = 0.0F;
  }

  if ((intfreeze == 0) && (intReset == 0)) {
    if (0.0F < intLim[0]) {
      ie[0] = setp[0] * dt;
    }

    if (0.0F < intLim[1]) {
      ie[1] = setp[2] * dt;
    }

    if (0.0F < intLim[2]) {
      ie[2] = setp[4] * dt;
    }
  }

  /* coder.ceval('printf','6\n'); */
  /* % position controller */
  for (i = 0; i < 3; i++) {
    b_K[i] = 0.0F;
    for (i0 = 0; i0 < 10; i0++) {
      b_K[i] += K[i + 3 * i0] * setp[i0];
    }

    F_desProp[i] = -b_K[i];

    /* % superposition of the desired Forces */
    /* coder.ceval('printf','7\n'); */
    u_pre[i] = 0.0F;
    for (i0 = 0; i0 < 3; i0++) {
      u_pre[i] += Kint[i + 3 * i0] * ie[i0];
    }
  }

  F_desInt[0] = -u_pre[0];
  F_desInt[1] = -u_pre[1];
  F_desInt[2] = -u_pre[2];
  ff[0] = setpAccRaw[0];
  ff[1] = setpAccRaw[1];
  ff[2] = setpAccRaw[2];
  for (i = 0; i < 3; i++) {
    st = ((F_desProp[i] + F_global[i]) + F_desInt[i]) + ff[i] * mass;

    /* u(1:3)=R_B*(F_total); */
    u[i] = st;
    u_pre[i] = st;
  }

  /* u(1:3)=Ryaw'*(F_total); */
  u[2] = u_pre[2];
  for (i = 0; i < 3; i++) {
    ie[i] = u[i];
  }

  limit_u(ie, uLim, u);

  /*  applying the control output limit */
}

void positionLQGC_initialize()
{
  rt_InitInfAndNaN(8U);
  positionLQGC_init();
}

void positionLQGC_terminate()
{
  /* (no terminate code required) */
}

/* End of code generation (positionLQGC.cpp) */
