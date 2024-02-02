//
// File: navigasinonlinirham.cpp
//
// Code generated for Simulink model 'navigasinonlinirham'.
//
// Model version                  : 1.24
// Simulink Coder version         : 8.10 (R2016a) 10-Feb-2016
// C/C++ source code generated on : Wed Sep 20 05:57:54 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "navigasinonlinirham.h"

// Private macros used by the generated code to access rtModel
#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

#ifndef rtmSetTPtr
# define rtmSetTPtr(rtm, val)          ((rtm)->Timing.t = (val))
#endif

// private model entry point functions
extern void navigasinonlinirham_derivatives();

//
// This function updates continuous states using the ODE3 fixed-step
// solver algorithm
//
void navigasinonlinirhamModelClass::rt_ertODEUpdateContinuousStates
  (RTWSolverInfo *si )
{
  // Solver Matrices
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 6;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  navigasinonlinirham_derivatives();

  // f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*));
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  this->step();
  navigasinonlinirham_derivatives();

  // f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*));
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  this->step();
  navigasinonlinirham_derivatives();

  // tnew = t + hA(3);
  // ynew = y + f*hB(:,3);
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

// Model step function
void navigasinonlinirhamModelClass::step()
{
  real_T vel[3];
  real_T K[15];
  real_T y[15];
  real_T B_0[9];
  int32_T r1;
  int32_T r2;
  int32_T r3;
  real_T a21;
  int32_T rtemp;
  int8_T I[25];
  static const int8_T b[5] = { 0, 0, 0, 0, 1 };

  static const real_T c[9] = { 0.0009192, 0.0, 0.0, 0.0, 0.0008847, 0.0, 0.0,
    0.0, 0.000447 };

  real_T rtb_Add1;
  real_T rtb_Add2;
  int8_T rtAction;
  real_T tmp[9];
  real_T tmp_0[5];
  real_T rtb_Add_0[3];
  real_T tmp_1[3];
  real_T I_0[25];
  real_T I_1[25];
  real_T tmp_2[3];
  real_T tmp_3[9];
  real_T tmp_4;
  if (rtmIsMajorTimeStep((&rtM))) {
    // set solver stop time
    rtsiSetSolverStopTime(&(&rtM)->solverInfo,(((&rtM)->Timing.clockTick0+1)*
      (&rtM)->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep((&rtM))) {
    (&rtM)->Timing.t[0] = rtsiGetT(&(&rtM)->solverInfo);
  }

  // Sum: '<S7>/Add2' incorporates:
  //   Inport: '<Root>/Inpawalx'
  //   Integrator: '<S7>/Integrator'

  rtb_Add2 = rtX.Integrator_CSTATE + rtU.Inpawalx;

  // Outport: '<Root>/Outlat'
  rtY.Outlat = rtb_Add2;

  // Sum: '<S7>/Add1' incorporates:
  //   Inport: '<Root>/Inpawaly'
  //   Integrator: '<S7>/Integrator1'

  rtb_Add1 = rtX.Integrator1_CSTATE + rtU.Inpawaly;

  // Outport: '<Root>/Outlong'
  rtY.Outlong = rtb_Add1;

  // Sum: '<S7>/Add' incorporates:
  //   Inport: '<Root>/Inpawalz'
  //   Integrator: '<S7>/Integrator2'

  rtDW.Add = rtX.Integrator2_CSTATE + rtU.Inpawalz;

  // If: '<S3>/If'
  if (rtmIsMajorTimeStep((&rtM))) {
    rtAction = (int8_T)!(rtDW.Add == 0.0);
    rtDW.If_ActiveSubsystem = rtAction;
  } else {
    rtAction = rtDW.If_ActiveSubsystem;
  }

  switch (rtAction) {
   case 0:
    // Outputs for IfAction SubSystem: '<S3>/If Action Subsystem' incorporates:
    //   ActionPort: '<S12>/Action Port'

    // Outport: '<Root>/Outz' incorporates:
    //   Inport: '<Root>/Inz'
    //   Inport: '<S12>/In1'

    rtY.Outz = rtU.Inz;

    // End of Outputs for SubSystem: '<S3>/If Action Subsystem'
    break;

   case 1:
    // Outputs for IfAction SubSystem: '<S3>/If Action Subsystem1' incorporates:
    //   ActionPort: '<S13>/Action Port'

    if (rtmIsMajorTimeStep((&rtM))) {
      // Memory: '<S13>/Memory1'
      rtDW.Memory1 = rtDW.Memory1_PreviousInput;

      // Memory: '<S13>/Memory'
      rtDW.Memory = rtDW.Memory_PreviousInput;
    }

    // Sum: '<S13>/Sum' incorporates:
    //   Gain: '<S13>/Gain'
    //   Gain: '<S13>/Gain1'
    //   Inport: '<Root>/Inz'
    //   Sum: '<S13>/Add'

    rtDW.Sum = ((rtDW.Memory1 + rtDW.Add) - rtDW.Memory) * 0.019900990099009919
      + 0.99009900990099009 * rtU.Inz;

    // Outport: '<Root>/Outz' incorporates:
    //   SignalConversion: '<S13>/OutportBufferForOut2'

    rtY.Outz = rtDW.Sum;

    // End of Outputs for SubSystem: '<S3>/If Action Subsystem1'
    break;
  }

  // End of If: '<S3>/If'

  // Outport: '<Root>/Outposx'
  rtY.Outposx = rtb_Add2;

  // Outport: '<Root>/Outposy'
  rtY.Outposy = rtb_Add1;

  // Outport: '<Root>/Outposz'
  rtY.Outposz = rtDW.Add;

  // MATLAB Function: '<S2>/MATLAB Function' incorporates:
  //   Inport: '<Root>/Inphi'
  //   Inport: '<Root>/Inteta'
  //   Inport: '<Root>/Inu'
  //   Inport: '<Root>/Inv'
  //   Inport: '<Root>/Inw'
  //   Inport: '<Root>/Inyaw'
  //   MATLAB Function: '<S6>/MATLAB Function'
  //   MATLAB Function: '<S6>/MATLAB Function1'
  //   MATLAB Function: '<S6>/MATLAB Function2'

  // MATLAB Function 'Navigasi/Model DVL/MATLAB Function': '<S16>:1'
  // '<S16>:1:3' Sf= 1;
  // resolusi sensor
  // '<S16>:1:4' bias=0.5229e-03;
  // '<S16>:1:5' deviasi=0.9587e-03;
  // '<S16>:1:6' ef=bias+deviasi;
  // '<S16>:1:7' uDVL=(u*(1+Sf))+ef;
  // MATLAB Function 'Navigasi/Model DVL/MATLAB Function1': '<S17>:1'
  // '<S17>:1:3' Sf= 1;
  // resolusi sensor
  // '<S17>:1:4' bias=0.2514e-03;
  // '<S17>:1:5' deviasi=0.9406e-03;
  // '<S17>:1:6' ef=bias+deviasi;
  // '<S17>:1:7' vDVL=(v*(1+Sf))+ef;
  // MATLAB Function 'Navigasi/Model DVL/MATLAB Function2': '<S18>:1'
  // '<S18>:1:3' Sf= 1;
  // resolusi sensor
  // '<S18>:1:4' bias=0.3065;
  // '<S18>:1:5' deviasi=0.6686;
  // '<S18>:1:6' ef=bias+deviasi;
  // '<S18>:1:7' wDVL=(w*(1+Sf))+ef;
  // MATLAB Function 'Navigasi/ Transformasi koordinat DVL/MATLAB Function': '<S11>:1' 
  // '<S11>:1:3' a11 = cos(yaw)*cos(teta);
  // '<S11>:1:4' a12 = cos(yaw)*sin(teta)*sin(phi)-sin(yaw)*cos(phi);
  // '<S11>:1:5' a13 = cos(yaw)*sin(teta)*cos(phi)+sin(yaw)*sin(phi);
  // '<S11>:1:6' a21 = sin(yaw)*cos(teta);
  // '<S11>:1:7' a22 = sin(yaw)*sin(teta)*sin(phi)+cos(yaw)*cos(phi);
  // '<S11>:1:8' a23 = sin(yaw)*sin(teta)*cos(phi)-cos(yaw)*sin(phi);
  // '<S11>:1:9' a31 = -sin(teta);
  // '<S11>:1:10' a32 = cos(teta)*sin(phi);
  // '<S11>:1:11' a33 = cos(teta)*cos(phi);
  // '<S11>:1:12' A = [a11 a12 a13;a21 a22 a23;a31 a32 a33];
  // '<S11>:1:13' vel = A*[u v w]';
  rtb_Add1 = rtU.Inu * 2.0 + 0.0014816;
  a21 = rtU.Inv * 2.0 + 0.001192;
  tmp_4 = rtU.Inw * 2.0 + 0.9751;
  tmp[0] = cos(rtU.Inyaw) * cos(rtU.Inteta);
  tmp[3] = cos(rtU.Inyaw) * sin(rtU.Inteta) * sin(rtU.Inphi) - sin(rtU.Inyaw) *
    cos(rtU.Inphi);
  tmp[6] = cos(rtU.Inyaw) * sin(rtU.Inteta) * cos(rtU.Inphi) + sin(rtU.Inyaw) *
    sin(rtU.Inphi);
  tmp[1] = sin(rtU.Inyaw) * cos(rtU.Inteta);
  tmp[4] = sin(rtU.Inyaw) * sin(rtU.Inteta) * sin(rtU.Inphi) + cos(rtU.Inyaw) *
    cos(rtU.Inphi);
  tmp[7] = sin(rtU.Inyaw) * sin(rtU.Inteta) * cos(rtU.Inphi) - cos(rtU.Inyaw) *
    sin(rtU.Inphi);
  tmp[2] = -sin(rtU.Inteta);
  tmp[5] = cos(rtU.Inteta) * sin(rtU.Inphi);
  tmp[8] = cos(rtU.Inteta) * cos(rtU.Inphi);
  for (r1 = 0; r1 < 3; r1++) {
    vel[r1] = tmp[r1 + 6] * tmp_4 + (tmp[r1 + 3] * a21 + tmp[r1] * rtb_Add1);
  }

  // MATLAB Function: '<S5>/MATLAB Function' incorporates:
  //   Integrator: '<S4>/Integrator'
  //   Integrator: '<S4>/Integrator1'
  //   Integrator: '<S4>/Integrator2'
  //   MATLAB Function: '<S2>/MATLAB Function'
  //   Sum: '<S9>/Add'

  // '<S11>:1:14' vN = vel(1);
  // '<S11>:1:15' vE = vel(2);
  // '<S11>:1:16' vD = vel(3);
  // end
  //  % Inizialization
  //  rad = 6378137; %WGS84 Equatorial radius in meters
  //  g = 9.7803267715; %m/s^2
  //  dt=1;
  //  H=[1 0 0 0 0
  //     0 0 1 0 0
  //     0 0 0 0 1];                % Observation Matrix
  //
  //  A=[ 1       g*dt 0      0    0
  //      -dt/rad 1    0      0    0
  //      0       0    1      g*dt 0
  //      0       0    dt/rad 1    0
  //      0       0    0      0    1];  % State Model
  //  Q=0;
  //  R=[0.9192;0.8847;0.447];
  //
  //  % simulated variables
  //  x=[dvN;attE;dvE;attN;dvD];
  //  % collect data
  //  dVN=x(1);
  //  dVE=x(2);
  //  dVD=x(3);
  //  Define storage for the variables that need to persist
  //  between time periods.
  // MATLAB Function 'Navigasi/Kalman Filter/MATLAB Function': '<S15>:1'
  // '<S15>:1:27' meas=[dvlN;dvlE;dvlD];
  // B
  // '<S15>:1:29' if isempty(P)
  if (!rtDW.P_not_empty) {
    //  First time through the code so do some initialization
    // '<S15>:1:31' xhat = zeros(5,1);
    // '<S15>:1:32' P = eye(5);
    memset(&rtDW.P_k[0], 0, 25U * sizeof(real_T));
    rtDW.P_not_empty = true;

    // '<S15>:1:33' radius = 6378137;
    // WGS84 Equatorial radius in meters
    // '<S15>:1:34' g = 9.7803267715;
    // m/s^2
    // '<S15>:1:35' dt=1;
    // '<S15>:1:36' A=[ 1       g*dt  0         0    0
    // '<S15>:1:37'      -dt/radius 1    0         0    0
    // '<S15>:1:38'      0          0    1         g*dt 0
    // '<S15>:1:39'      0          0    dt/radius 1    0
    // '<S15>:1:40'      0          0    0         0    1];
    rtDW.A[0] = 1.0;
    rtDW.A[5] = rtDW.g * rtDW.dt;
    rtDW.A[10] = 0.0;
    rtDW.A[15] = 0.0;
    rtDW.A[20] = 0.0;
    rtDW.A[1] = -rtDW.dt / rtDW.radius;
    rtDW.A[6] = 1.0;
    rtDW.A[11] = 0.0;
    rtDW.A[16] = 0.0;
    rtDW.A[21] = 0.0;
    rtDW.A[2] = 0.0;
    rtDW.A[7] = 0.0;
    rtDW.A[12] = 1.0;
    rtDW.A[17] = rtDW.g * rtDW.dt;
    rtDW.A[22] = 0.0;
    rtDW.A[3] = 0.0;
    rtDW.A[8] = 0.0;
    rtDW.A[13] = rtDW.dt / rtDW.radius;
    rtDW.A[18] = 1.0;
    rtDW.A[23] = 0.0;
    for (r1 = 0; r1 < 5; r1++) {
      rtDW.P_k[r1 + 5 * r1] = 1.0;
      rtDW.A[4 + 5 * r1] = b[r1];
    }

    //  State Model
    // '<S15>:1:41' C = [1 0 0 0 0
    // '<S15>:1:42'         0 0 1 0 0
    // '<S15>:1:43'         0 0 0 0 1];
    // '<S15>:1:44' Q=0;
    // '<S15>:1:45' R=[0.9192e-03 0 0;0 0.8847e-03 0;0 0 0.447e-03];
    // '<S15>:1:46' R=sqrt(R)
    for (r1 = 0; r1 < 9; r1++) {
      rtDW.R[r1] = c[r1];
      rtDW.R[r1] = sqrt(rtDW.R[r1]);
    }
  }

  //  Propagate the state estimate covariance matrix:
  // '<S15>:1:49' xhat = A*xhat;
  for (r1 = 0; r1 < 5; r1++) {
    tmp_0[r1] = 0.0;
    for (r2 = 0; r2 < 5; r2++) {
      tmp_0[r1] += rtDW.A[5 * r2 + r1] * rtDW.xhat[r2];
    }
  }

  // covariance matrix:
  // '<S15>:1:51' P = A*P*A' + Q;
  for (r1 = 0; r1 < 5; r1++) {
    rtDW.xhat[r1] = tmp_0[r1];
    for (r2 = 0; r2 < 5; r2++) {
      I_0[r1 + 5 * r2] = 0.0;
      for (r3 = 0; r3 < 5; r3++) {
        I_0[r1 + 5 * r2] += rtDW.A[5 * r3 + r1] * rtDW.P_k[5 * r2 + r3];
      }
    }
  }

  for (r1 = 0; r1 < 5; r1++) {
    for (r2 = 0; r2 < 5; r2++) {
      rtb_Add1 = 0.0;
      for (r3 = 0; r3 < 5; r3++) {
        rtb_Add1 += I_0[5 * r3 + r1] * rtDW.A[5 * r3 + r2];
      }

      rtDW.P_k[r1 + 5 * r2] = rtb_Add1 + rtDW.Q;
    }
  }

  //  Calculate the Kalman gain
  // '<S15>:1:53' K = P*C'/(C*P*C' + R);
  for (r1 = 0; r1 < 5; r1++) {
    for (r2 = 0; r2 < 3; r2++) {
      y[r1 + 5 * r2] = 0.0;
      for (r3 = 0; r3 < 5; r3++) {
        y[r1 + 5 * r2] += rtDW.P_k[5 * r3 + r1] * rtDW.C[3 * r3 + r2];
      }
    }
  }

  for (r1 = 0; r1 < 3; r1++) {
    for (r2 = 0; r2 < 5; r2++) {
      K[r1 + 3 * r2] = 0.0;
      for (r3 = 0; r3 < 5; r3++) {
        K[r1 + 3 * r2] += rtDW.C[3 * r3 + r1] * rtDW.P_k[5 * r2 + r3];
      }
    }
  }

  for (r1 = 0; r1 < 3; r1++) {
    for (r2 = 0; r2 < 3; r2++) {
      rtb_Add1 = 0.0;
      for (r3 = 0; r3 < 5; r3++) {
        rtb_Add1 += K[3 * r3 + r1] * rtDW.C[3 * r3 + r2];
      }

      B_0[r1 + 3 * r2] = rtDW.R[3 * r2 + r1] + rtb_Add1;
    }
  }

  r1 = 0;
  r2 = 1;
  r3 = 2;
  rtb_Add1 = fabs(B_0[0]);
  a21 = fabs(B_0[1]);
  if (a21 > rtb_Add1) {
    rtb_Add1 = a21;
    r1 = 1;
    r2 = 0;
  }

  if (fabs(B_0[2]) > rtb_Add1) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  B_0[r2] /= B_0[r1];
  B_0[r3] /= B_0[r1];
  B_0[3 + r2] -= B_0[3 + r1] * B_0[r2];
  B_0[3 + r3] -= B_0[3 + r1] * B_0[r3];
  B_0[6 + r2] -= B_0[6 + r1] * B_0[r2];
  B_0[6 + r3] -= B_0[6 + r1] * B_0[r3];
  if (fabs(B_0[3 + r3]) > fabs(B_0[3 + r2])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  B_0[3 + r3] /= B_0[3 + r2];
  B_0[6 + r3] -= B_0[3 + r3] * B_0[6 + r2];
  for (rtemp = 0; rtemp < 5; rtemp++) {
    K[rtemp + 5 * r1] = y[rtemp] / B_0[r1];
    K[rtemp + 5 * r2] = y[5 + rtemp] - K[5 * r1 + rtemp] * B_0[3 + r1];
    K[rtemp + 5 * r3] = y[10 + rtemp] - K[5 * r1 + rtemp] * B_0[6 + r1];
    K[rtemp + 5 * r2] /= B_0[3 + r2];
    K[rtemp + 5 * r3] -= K[5 * r2 + rtemp] * B_0[6 + r2];
    K[rtemp + 5 * r3] /= B_0[6 + r3];
    K[rtemp + 5 * r2] -= K[5 * r3 + rtemp] * B_0[3 + r3];
    K[rtemp + 5 * r1] -= K[5 * r3 + rtemp] * B_0[r3];
    K[rtemp + 5 * r1] -= K[5 * r2 + rtemp] * B_0[r2];
  }

  //  Calculate the measurement residual
  // '<S15>:1:55' resid = meas - C*xhat;
  //  Update the state and error covariance estimate
  // '<S15>:1:57' xhat = xhat + K*resid;
  rtb_Add_0[0] = rtX.Integrator_CSTATE_a - vel[0];
  rtb_Add_0[1] = rtX.Integrator1_CSTATE_g - vel[1];
  rtb_Add_0[2] = rtX.Integrator2_CSTATE_o - vel[2];
  for (r1 = 0; r1 < 3; r1++) {
    tmp_1[r1] = 0.0;
    for (r2 = 0; r2 < 5; r2++) {
      tmp_1[r1] += rtDW.C[3 * r2 + r1] * rtDW.xhat[r2];
    }

    vel[r1] = rtb_Add_0[r1] - tmp_1[r1];
  }

  for (r1 = 0; r1 < 5; r1++) {
    rtDW.xhat[r1] += (K[r1 + 5] * vel[1] + K[r1] * vel[0]) + K[r1 + 10] * vel[2];
  }

  // '<S15>:1:58' P = (eye(size(K,1))-K*C)*P;
  for (r1 = 0; r1 < 25; r1++) {
    I[r1] = 0;
  }

  for (r1 = 0; r1 < 5; r1++) {
    I[r1 + 5 * r1] = 1;
  }

  for (r1 = 0; r1 < 5; r1++) {
    for (r2 = 0; r2 < 5; r2++) {
      I_0[r1 + 5 * r2] = (real_T)I[5 * r2 + r1] - ((rtDW.C[3 * r2 + 1] * K[r1 +
        5] + rtDW.C[3 * r2] * K[r1]) + rtDW.C[3 * r2 + 2] * K[r1 + 10]);
    }
  }

  for (r1 = 0; r1 < 5; r1++) {
    for (r2 = 0; r2 < 5; r2++) {
      I_1[r1 + 5 * r2] = 0.0;
      for (r3 = 0; r3 < 5; r3++) {
        I_1[r1 + 5 * r2] += I_0[5 * r3 + r1] * rtDW.P_k[5 * r2 + r3];
      }
    }
  }

  for (r1 = 0; r1 < 5; r1++) {
    for (r2 = 0; r2 < 5; r2++) {
      rtDW.P_k[r2 + 5 * r1] = I_1[5 * r1 + r2];
    }
  }

  // Sum: '<S10>/Add' incorporates:
  //   Integrator: '<S4>/Integrator'
  //   Integrator: '<S4>/Integrator1'
  //   Integrator: '<S4>/Integrator2'

  //  Post the results
  // '<S15>:1:60' xhatOut = xhat;
  // '<S15>:1:61' yhatOut = C*xhatOut;
  tmp_2[0] = rtX.Integrator_CSTATE_a;
  tmp_2[1] = rtX.Integrator1_CSTATE_g;
  tmp_2[2] = rtX.Integrator2_CSTATE_o;
  for (r1 = 0; r1 < 3; r1++) {
    // MATLAB Function: '<S5>/MATLAB Function' incorporates:
    //   Sum: '<S10>/Add'

    tmp_1[r1] = 0.0;
    for (r2 = 0; r2 < 5; r2++) {
      tmp_1[r1] += rtDW.C[3 * r2 + r1] * rtDW.xhat[r2];
    }

    // Sum: '<S10>/Add'
    vel[r1] = tmp_2[r1] - tmp_1[r1];
  }

  // Outport: '<Root>/Outkecx '
  rtY.Outkecx = vel[0];

  // Outport: '<Root>/Outkecy'
  rtY.Outkecy = vel[1];

  // Outport: '<Root>/Outkecz'
  rtY.Outkecz = vel[2];

  // MATLAB Function: '<S8>/MATLAB Function' incorporates:
  //   Inport: '<Root>/Inphi'
  //   Inport: '<Root>/Inteta'
  //   Inport: '<Root>/Inudot'
  //   Inport: '<Root>/Invdot'
  //   Inport: '<Root>/Inwdot'
  //   Inport: '<Root>/Inyaw'

  // MATLAB Function 'Navigasi/Transformasi koordinat IMU/MATLAB Function': '<S20>:1' 
  // '<S20>:1:4' a11=cos(teta)*cos(yaw);
  // '<S20>:1:5' a12=cos(teta)*sin(yaw);
  // '<S20>:1:6' a13=-sin(teta);
  // '<S20>:1:7' a21=sin(phi)*sin(teta)*cos(yaw);
  // '<S20>:1:8' a22=sin(phi)*sin(teta)*sin(yaw)+cos(phi)*cos(yaw);
  // '<S20>:1:9' a23=sin(phi)*cos(teta);
  // '<S20>:1:10' a31=cos(phi)*sin(teta)*cos(yaw)+sin(phi)*sin(yaw);
  // '<S20>:1:11' a32=cos(phi)*sin(teta)*sin(yaw)-sin(phi)*cos(yaw);
  // '<S20>:1:12' a33=cos(phi)*cos(teta);
  // '<S20>:1:13' A=[a11 a12 a13;a21 a22 a23;a31 a32 a33];
  // '<S20>:1:14' ned=A*[udot vdot wdot]';
  tmp_3[0] = cos(rtU.Inteta) * cos(rtU.Inphi);
  tmp_3[3] = cos(rtU.Inteta) * sin(rtU.Inphi);
  tmp_3[6] = -sin(rtU.Inteta);
  tmp_3[1] = sin(rtU.Inyaw) * sin(rtU.Inteta) * cos(rtU.Inphi);
  tmp_3[4] = sin(rtU.Inyaw) * sin(rtU.Inteta) * sin(rtU.Inphi) + cos(rtU.Inyaw) *
    cos(rtU.Inphi);
  tmp_3[7] = sin(rtU.Inyaw) * cos(rtU.Inteta);
  tmp_3[2] = cos(rtU.Inyaw) * sin(rtU.Inteta) * cos(rtU.Inphi) + sin(rtU.Inyaw) *
    sin(rtU.Inphi);
  tmp_3[5] = cos(rtU.Inyaw) * sin(rtU.Inteta) * sin(rtU.Inphi) - sin(rtU.Inyaw) *
    cos(rtU.Inphi);
  tmp_3[8] = cos(rtU.Inyaw) * cos(rtU.Inteta);
  for (r1 = 0; r1 < 3; r1++) {
    rtb_Add_0[r1] = tmp_3[r1 + 6] * rtU.Inwdot + (tmp_3[r1 + 3] * rtU.Invdot +
      tmp_3[r1] * rtU.Inudot);
  }

  // MATLAB Function: '<S4>/MATLAB Function' incorporates:
  //   Integrator: '<S4>/Integrator'
  //   Integrator: '<S4>/Integrator1'
  //   Integrator: '<S4>/Integrator2'
  //   MATLAB Function: '<S8>/MATLAB Function'

  // '<S20>:1:15' udotned=ned(1);
  // '<S20>:1:16' vdotned=ned(2);
  // '<S20>:1:17' wdotned=ned(3);
  // MATLAB Function 'Navigasi/IMU/MATLAB Function': '<S14>:1'
  // '<S14>:1:4' omegaE = 7.292115E-5;
  // kecepatan rotasi bumi
  // '<S14>:1:5' R = 6378137;
  // jari-jari bumi dalam model bola
  // '<S14>:1:6' aimuN=udot-2*omegaE*sin(latitude)*vE-(vE)^2/R*tan(latitude)+vN*vD/R; 
  rtDW.aimuN = ((rtb_Add_0[0] - 0.0001458423 * sin(rtb_Add2) *
                 rtX.Integrator1_CSTATE_g) - rtX.Integrator1_CSTATE_g *
                rtX.Integrator1_CSTATE_g / 6.378137E+6 * tan(rtb_Add2)) +
    rtX.Integrator_CSTATE_a * rtX.Integrator2_CSTATE_o / 6.378137E+6;

  // '<S14>:1:7' aimuE=vdot-2*omegaE*sin(latitude)*vN-vE*vN/R*tan(latitude)-2*omegaE*cos(latitude)*vD-vE*vD/R; 
  rtDW.aimuE = (((rtb_Add_0[1] - 0.0001458423 * sin(rtb_Add2) *
                  rtX.Integrator_CSTATE_a) - rtX.Integrator1_CSTATE_g *
                 rtX.Integrator_CSTATE_a / 6.378137E+6 * tan(rtb_Add2)) -
                0.0001458423 * cos(rtb_Add2) * rtX.Integrator2_CSTATE_o) -
    rtX.Integrator1_CSTATE_g * rtX.Integrator2_CSTATE_o / 6.378137E+6;

  // '<S14>:1:8' aimuD=wdot-2*omegaE*cos(latitude)*vE-(vE)^2/R-(vN)^2/R;
  rtDW.aimuD = ((rtb_Add_0[2] - 0.0001458423 * cos(rtb_Add2) *
                 rtX.Integrator1_CSTATE_g) - rtX.Integrator1_CSTATE_g *
                rtX.Integrator1_CSTATE_g / 6.378137E+6) -
    rtX.Integrator_CSTATE_a * rtX.Integrator_CSTATE_a / 6.378137E+6;

  // MATLAB Function: '<S7>/MATLAB Function'
  // end
  // MATLAB Function 'Navigasi/Perhitungan Posisi NED/MATLAB Function': '<S19>:1' 
  // '<S19>:1:3' R = 6378137;
  // '<S19>:1:4' lat=vN/R;
  rtDW.lat = vel[0] / 6.378137E+6;

  // '<S19>:1:5' long=vE/R*cos(lat_est);
  rtDW.long_a = vel[1] / 6.378137E+6 * cos(rtb_Add2);

  // '<S19>:1:6' depth=vD;
  rtDW.depth = vel[2];
  if (rtmIsMajorTimeStep((&rtM))) {
    // Update for IfAction SubSystem: '<S3>/If Action Subsystem1' incorporates:
    //   Update for ActionPort: '<S13>/Action Port'

    // Update for If: '<S3>/If'
    if ((rtDW.If_ActiveSubsystem == 1) && rtmIsMajorTimeStep((&rtM))) {
      // Update for Memory: '<S13>/Memory1'
      rtDW.Memory1_PreviousInput = rtDW.Sum;

      // Update for Memory: '<S13>/Memory'
      rtDW.Memory_PreviousInput = rtDW.Add;
    }

    // End of Update for If: '<S3>/If'
    // End of Update for SubSystem: '<S3>/If Action Subsystem1'
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep((&rtM))) {
    rt_ertODEUpdateContinuousStates(&(&rtM)->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++(&rtM)->Timing.clockTick0;
    (&rtM)->Timing.t[0] = rtsiGetSolverStopTime(&(&rtM)->solverInfo);

    {
      // Update absolute timer for sample time: [20.0s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 20.0, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      (&rtM)->Timing.clockTick1++;
    }
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void navigasinonlinirhamModelClass::navigasinonlinirham_derivatives()
{
  XDot *_rtXdot;
  _rtXdot = ((XDot *) (&rtM)->ModelData.derivs);

  // Derivatives for Integrator: '<S7>/Integrator'
  _rtXdot->Integrator_CSTATE = rtDW.lat;

  // Derivatives for Integrator: '<S7>/Integrator1'
  _rtXdot->Integrator1_CSTATE = rtDW.long_a;

  // Derivatives for Integrator: '<S7>/Integrator2'
  _rtXdot->Integrator2_CSTATE = rtDW.depth;

  // Derivatives for Integrator: '<S4>/Integrator'
  _rtXdot->Integrator_CSTATE_a = rtDW.aimuN;

  // Derivatives for Integrator: '<S4>/Integrator1'
  _rtXdot->Integrator1_CSTATE_g = rtDW.aimuE;

  // Derivatives for Integrator: '<S4>/Integrator2'
  _rtXdot->Integrator2_CSTATE_o = rtDW.aimuD;
}

// Model initialize function
void navigasinonlinirhamModelClass::initialize()
{
  // Registration code
  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&(&rtM)->solverInfo, &(&rtM)->Timing.simTimeStep);
    rtsiSetTPtr(&(&rtM)->solverInfo, &rtmGetTPtr((&rtM)));
    rtsiSetStepSizePtr(&(&rtM)->solverInfo, &(&rtM)->Timing.stepSize0);
    rtsiSetdXPtr(&(&rtM)->solverInfo, &(&rtM)->ModelData.derivs);
    rtsiSetContStatesPtr(&(&rtM)->solverInfo, (real_T **) &(&rtM)
                         ->ModelData.contStates);
    rtsiSetNumContStatesPtr(&(&rtM)->solverInfo, &(&rtM)->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&rtM)->solverInfo, &(&rtM)
      ->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&rtM)->solverInfo, &(&rtM)
      ->ModelData.periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&rtM)->solverInfo, &(&rtM)
      ->ModelData.periodicContStateRanges);
    rtsiSetErrorStatusPtr(&(&rtM)->solverInfo, (&rtmGetErrorStatus((&rtM))));
    rtsiSetRTModelPtr(&(&rtM)->solverInfo, (&rtM));
  }

  rtsiSetSimTimeStep(&(&rtM)->solverInfo, MAJOR_TIME_STEP);
  (&rtM)->ModelData.intgData.y = (&rtM)->ModelData.odeY;
  (&rtM)->ModelData.intgData.f[0] = (&rtM)->ModelData.odeF[0];
  (&rtM)->ModelData.intgData.f[1] = (&rtM)->ModelData.odeF[1];
  (&rtM)->ModelData.intgData.f[2] = (&rtM)->ModelData.odeF[2];
  (&rtM)->ModelData.contStates = ((X *) &rtX);
  rtsiSetSolverData(&(&rtM)->solverInfo, (void *)&(&rtM)->ModelData.intgData);
  rtsiSetSolverName(&(&rtM)->solverInfo,"ode3");
  rtmSetTPtr((&rtM), &(&rtM)->Timing.tArray[0]);
  (&rtM)->Timing.stepSize0 = 20.0;

  {
    static const int8_T tmp[15] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1
    };

    int32_T i;

    // Start for If: '<S3>/If'
    rtDW.If_ActiveSubsystem = -1;

    // InitializeConditions for Integrator: '<S7>/Integrator'
    rtX.Integrator_CSTATE = 0.0;

    // InitializeConditions for Integrator: '<S7>/Integrator1'
    rtX.Integrator1_CSTATE = 0.0;

    // InitializeConditions for Integrator: '<S7>/Integrator2'
    rtX.Integrator2_CSTATE = 0.0;

    // InitializeConditions for Integrator: '<S4>/Integrator'
    rtX.Integrator_CSTATE_a = 0.0;

    // InitializeConditions for Integrator: '<S4>/Integrator1'
    rtX.Integrator1_CSTATE_g = 0.0;

    // InitializeConditions for Integrator: '<S4>/Integrator2'
    rtX.Integrator2_CSTATE_o = 0.0;

    // SystemInitialize for MATLAB Function: '<S5>/MATLAB Function'
    for (i = 0; i < 15; i++) {
      rtDW.C[i] = tmp[i];
    }

    rtDW.radius = 6.378137E+6;
    rtDW.g = 9.7803267715;
    rtDW.dt = 1.0;

    // End of SystemInitialize for MATLAB Function: '<S5>/MATLAB Function'
  }
}

// Constructor
navigasinonlinirhamModelClass::navigasinonlinirhamModelClass()
{
}

// Destructor
navigasinonlinirhamModelClass::~navigasinonlinirhamModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL * navigasinonlinirhamModelClass::getRTM()
{
  return (&rtM);
}

//
// File trailer for generated code.
//
// [EOF]
//
