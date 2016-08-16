/*
 * variable.c
 *
 *  Created on: 2016. 7. 18.
 *      Author: jgkim2020
 *
 * I do not take any responsibility for any consequences resulting from the
 * use of this software. USE AT YOUR OWN RISK!
 *
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "myvariable.h"

/******************************************************************************
 *
 * Register
 *
 ******************************************************************************/



/******************************************************************************
 *
 * Parameter
 *
 ******************************************************************************/

// System parameters
int i;
volatile int Tick1000Hz = 0;
volatile int seconds = 0;
int Tickloop = 0;
int TickLogStart, TickLogFinish;
float dt = OPERATING_PERIOD/1000.0;
int PWMFreq = 1000;
int PWMLoad = 0;

// Parameters for sensor readings
uint8_t raw_byte[18];
uint8_t *raw_byte_a, *raw_byte_w, *raw_byte_b;
int16_t raw_data[10];
int16_t *raw_a, *raw_w, *raw_b;
int16_t raw_data_offset[9];
int16_t *raw_offset_a, *raw_offset_w, *raw_offset_b;
uint32_t raw_v = 0; // voltage
float data[28];
float *a, *w, *b, *euler, *quaternion, *quaternion_acc, *quaternion_mag;
float offset_a[3], offset_w[3];
float cos_calib, quaternion_calib[4];

// Parameters for Kalman filter
float F[18] = {4, 4, }; // state-transition model
float H[18] = {4, 4, }; // 4x4 identity matrix
float K[18] = {4, 4, }; // optimal Kalman gain
float P[18] = {4, 4, }; // estimate covariance
float Pp[18] = {4, 4, }; // a priori state estimate covariance
float Q[18] = {4, 4, }; // process noise covariance
float R[18] = {4, 4, }; // observation noise covariance
float M3x3buffer[11] = {3, 3, };
float M4x1buffer1[6] = {4, 1, };
float M4x1buffer2[6] = {4, 1, };
float M4x4buffer1[18] = {4, 4, };
float M4x4buffer2[18] = {4, 4, };
float M4x4buffer3[18] = {4, 4, };
float x[6] = {4, 1, }; // state
float xp[6] = {4, 1, }; // a priori state estimate
float z[6] = {4, 1, }; // quaternion
float eulerK[3], quaternionK[4];
float I_hat[5] = {3, 1, };
float J_hat[5] = {3, 1, };
float K_hat[5] = {3, 1, };

// Parameters for PID
float euler_set[3] = {0, 0, 0};
float yaw_set_delta = 0.0;
float quaternion_set[4];
float quaternion_delta[4];
float angle_delta;
float prev_angle_delta = 0;
float w_req[3];
float torque_req[3];
float thrust_delta[4];
float thrust_base = 0.0; // base thrust (in gf) of each rotor
float thrust[4];
float p1 = 10.0;
float d1 = 0.0;
float p2 = 55.0;
float l2 = 50.0;

// Parameters for command handler
volatile int XBEEIntflag = 0;
volatile int XBEEIntmsg_pointer = 0;
volatile char XBEEIntmsg[XBEEIntmsg_size];
int STATUS_startup = 1;
int STATUS_loop = 1;
int STATUS_stoploop = 0;
int STATUS_CTRL_h = 0;
int STATUS_CTRL_v = 0;
int STATUS_CTRL_p = 0;
int STATUS_CTRL_X = 0;
int STATUS_CTRL_w = 0;
int STATUS_CTRL_s = 0;
int STATUS_CTRL_ad = -1;
int STATUS_CTRL_58 = -1;
int STATUS_CTRL_46 = -1;
