//
// File: attitudeController.cpp
//
// MATLAB Coder version            : 2.7
// C/C++ source code generated on  : 16-Dec-2014 15:07:04
//

// Include Files
#include "rt_nonfinite.h"
#include "attitudeController.h"

// Variable Definitions
static float ie[2];

// Function Declarations
static void attitudeController_init();
static float norm(const float x[3]);

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
static void attitudeController_init()
{
  int i;
  for (i = 0; i < 2; i++) {
    ie[i] = 0.0F;
  }
}

//
// Arguments    : const float x[3]
// Return Type  : float
//
static float norm(const float x[3])
{
  float y;
  float scale;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.17549435E-38F;
  for (k = 0; k < 3; k++) {
    absxk = (real32_T)fabs((real_T)x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * (real32_T)sqrt((real_T)y);
}

//
// R_IB: 3x3, rotation matrix (attitude)
//  yaw: scalar, current yaw angle in rad
//  yawOffset: unused atm
//  F_des: 3x1, desired force vector in NED
//  Kp: 3x1, P control gains [roll; pitch; yaw]
//  Kd: 3x1, D control gains [roll; pitch; yaw]
//  Ki: 3x1, I control gains [roll; pitch; yaw]
//  e_Rd_v: 6x1, ?
//  intFreeze: bool, if true the controller stops integrating
//  intReset: bool, if true the intgerators are reset
// Arguments    : const float R_IB[9]
//                float yaw
//                float yawOffset
//                float F_des[3]
//                const float Kp[3]
//                const float Kd[3]
//                const float Ki[3]
//                const float omega[3]
//                boolean_T intFreeze
//                boolean_T intReset
//                float rates_des[3]
//                float *thrust
// Return Type  : void
//
void attitudeController(const float R_IB[9], float yaw, float, float F_des[3],
  const float Kp[3], const float Kd[3], const float Ki[3], const float omega[3],
  boolean_T intFreeze, boolean_T intReset, float rates_des[3], float *thrust)
{
  float B;
  float z_B_des[3];
  int i;
  float x_C_idx_0;
  float x_C_idx_1;
  float b_z_B_des[3];
  float y_B_des[3];
  float b_y_B_des[3];
  float R_des[9];
  float b_R_des[9];
  float b_R_IB[9];
  int i0;
  int i1;

  // % init persistent variables
  //  dt
  // F_des=RYaw*F_des;
  // desired thrust in body frame
  if (F_des[2] <= -0.1F) {
    // thrust=(F_des'*z_b)/R(3,3);
    *thrust = -F_des[2] / R_IB[8];

    // Thrust=norm(F_des);
    // Thrust=F_des'*z_b;
  } else {
    F_des[2] = -0.1F;
    *thrust = 0.1F / R_IB[8];

    // Thrust=norm(F_des);
  }

  // desired body z axis, equal to force vector mirrored on rotor plane
  B = norm(F_des);
  for (i = 0; i < 3; i++) {
    z_B_des[i] = -F_des[i] / B;
  }

  // desired direction in world coordinates (Yaw angle)
  x_C_idx_0 = (real32_T)cos((real_T)yaw);
  x_C_idx_1 = (real32_T)sin((real_T)yaw);

  // desired body y axis
  b_z_B_des[0] = z_B_des[1] * 0.0F - z_B_des[2] * x_C_idx_1;
  b_z_B_des[1] = z_B_des[2] * x_C_idx_0 - z_B_des[0] * 0.0F;
  b_z_B_des[2] = z_B_des[0] * x_C_idx_1 - z_B_des[1] * x_C_idx_0;
  B = norm(b_z_B_des);
  y_B_des[0] = (z_B_des[1] * 0.0F - z_B_des[2] * x_C_idx_1) / B;
  y_B_des[1] = (z_B_des[2] * x_C_idx_0 - z_B_des[0] * 0.0F) / B;
  y_B_des[2] = (z_B_des[0] * x_C_idx_1 - z_B_des[1] * x_C_idx_0) / B;

  // desired body x axis
  // desired Rotation Matrix
  b_y_B_des[0] = y_B_des[1] * z_B_des[2] - y_B_des[2] * z_B_des[1];
  b_y_B_des[1] = y_B_des[2] * z_B_des[0] - y_B_des[0] * z_B_des[2];
  b_y_B_des[2] = y_B_des[0] * z_B_des[1] - y_B_des[1] * z_B_des[0];
  for (i = 0; i < 3; i++) {
    R_des[i] = b_y_B_des[i];
    R_des[3 + i] = y_B_des[i];
    R_des[6 + i] = z_B_des[i];
  }

  // % Attitude Controller
  //  P controller
  // error rotation matrix
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_R_des[i + 3 * i0] = 0.0F;
      for (i1 = 0; i1 < 3; i1++) {
        b_R_des[i + 3 * i0] += R_des[i1 + 3 * i] * R_IB[i1 + 3 * i0];
      }

      b_R_IB[i + 3 * i0] = 0.0F;
      for (i1 = 0; i1 < 3; i1++) {
        b_R_IB[i + 3 * i0] += R_IB[i1 + 3 * i] * R_des[i1 + 3 * i0];
      }
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      R_des[i0 + 3 * i] = 0.5F * (b_R_des[i0 + 3 * i] - b_R_IB[i0 + 3 * i]);
    }
  }

  // error rotation vector
  //  include an integral part
  if (intFreeze == 0) {
    if (intReset == 0) {
      if (*thrust > 2.5F) {
        if ((real32_T)fabs((real_T)ie[0]) < 5.0F) {
          ie[0] += R_des[7] * 0.02F;
        }

        if ((real32_T)fabs((real_T)ie[1]) < 5.0F) {
          ie[1] += -R_des[6] * 0.02F;
        }
      } else {
        for (i = 0; i < 2; i++) {
          ie[i] = 0.0F;
        }
      }
    } else {
      for (i = 0; i < 2; i++) {
        ie[i] = 0.0F;
      }
    }
  }

  //
  // e_Rd_v=[e_Rd(2,3);e_Rd(1,3);e_Rd(1,2)];
  // controll law
  // e_Rd_v=[e_Rd_v(1);e_Rd_v(3);(e_Rd_v(5))];
  rates_des[0] = (Kp[0] * R_des[7] + Kd[0] * omega[0]) + Ki[0] * ie[0];
  rates_des[1] = (Kp[1] * -R_des[6] + Kd[1] * omega[1]) + Ki[1] * ie[1];
  rates_des[2] = (Kp[2] * R_des[3] + Kd[2] * omega[2]) + Ki[2] * 0.0F;
}

//
// Arguments    : void
// Return Type  : void
//
void attitudeController_initialize()
{
  rt_InitInfAndNaN(8U);
  attitudeController_init();
}

//
// Arguments    : void
// Return Type  : void
//
void attitudeController_terminate()
{
  // (no terminate code required)
}

//
// File trailer for attitudeController.cpp
//
// [EOF]
//
