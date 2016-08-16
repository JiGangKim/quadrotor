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

/******************************************************************************
 *
 * Register
 *
 ******************************************************************************/

#define LED_PORT 				GPIO_PORTF_BASE
#define LED_R_PIN 				GPIO_PIN_1
#define LED_G_PIN 				GPIO_PIN_3
#define LED_B_PIN 				GPIO_PIN_2
#define SWITCH_PORT 			GPIO_PORTF_BASE
#define SWITCH1_PIN 			GPIO_PIN_4
#define SWITCH2_PIN 			GPIO_PIN_0

#define XBEE_PORT 				GPIO_PORTB_BASE // UART1
#define XBEE_POWER_PORT 		GPIO_PORTA_BASE
#define XBEE_RX_PIN 			GPIO_PIN_0
#define XBEE_TX_PIN 			GPIO_PIN_1
#define XBEE_POWER_PIN 			GPIO_PIN_3

#define I2C_PORT 				GPIO_PORTB_BASE // I2C0
#define I2C_SCL_PIN 			GPIO_PIN_2
#define I2C_SDA_PIN 			GPIO_PIN_3

#define ROTOR_POWER_PORT 		GPIO_PORTE_BASE
#define ROTOR1_PORT 			GPIO_PORTE_BASE // M0PWM4
#define ROTOR2_PORT 			GPIO_PORTB_BASE // M0PWM0
#define ROTOR3_PORT 			GPIO_PORTB_BASE // M0PWM2
#define ROTOR4_PORT 			GPIO_PORTC_BASE // M0PWM6
#define ROTOR_POWER_PIN 		GPIO_PIN_0
#define ROTOR1_PIN 				GPIO_PIN_4
#define ROTOR2_PIN 				GPIO_PIN_6
#define ROTOR3_PIN 				GPIO_PIN_4
#define ROTOR4_PIN 				GPIO_PIN_4

#define BATTERY_VOLTAGE_PORT 	GPIO_PORTE_BASE // ADC0
#define ULTRASONIC_TRIG_PORT 	GPIO_PORTD_BASE
#define ULTRASONIC_ECHO_PORT 	GPIO_PORTD_BASE
#define BATTERY_VOLTAGE_PIN 	GPIO_PIN_3
#define ULTRASONIC_TRIG_PIN 	GPIO_PIN_1
#define ULTRASONIC_ECHO_PIN 	GPIO_PIN_0

#define MPU6050_ADDRESS 		0x68
#define HMC5883L_ADDRESS 		0x1E

#define MPU6050_PWR				0x6B
#define MPU6050_ACC_CONFIG		0x1C
#define MPU6050_GYRO_CONFIG		0x1B
#define ACC_XOUT_H 				0x3B
#define ACC_XOUT_L 				0x3C
#define ACC_YOUT_H 				0x3D
#define ACC_YOUT_L 				0x3E
#define ACC_ZOUT_H 				0x3F
#define ACC_ZOUT_L 				0x40
#define GYRO_XOUT_H 			0x43
#define GYRO_XOUT_L 			0x44
#define GYRO_YOUT_H 			0x45
#define GYRO_YOUT_L 			0x46
#define GYRO_ZOUT_H 			0x47
#define GYRO_ZOUT_L 			0x48
#define HMC5883L_CONFIG_REGA 	0x00
#define HMC5883L_CONFIG_REGB 	0x01
#define HMC5883L_MODE_REG 		0x02
#define MAG_XOUT_H 				0x03
#define MAG_XOUT_L 				0x04
#define MAG_YOUT_H 				0x07
#define MAG_YOUT_L 				0x08
#define MAG_ZOUT_H 				0x05
#define MAG_ZOUT_L 				0x06

#define PI 						3.141592653589793238462643
#define OPERATING_PERIOD 		5

/******************************************************************************
 *
 * Parameter
 *
 ******************************************************************************/

// System parameters
extern int i;
extern volatile int Tick1000Hz;
extern volatile int seconds;
extern int Tickloop;
extern int TickLogStart, TickLogFinish;
extern float dt;
extern int PWMFreq;
extern int PWMLoad;

// Parameters for sensor readings
extern uint8_t raw_byte[18];
extern uint8_t *raw_byte_a, *raw_byte_w, *raw_byte_b;
extern int16_t raw_data[10];
extern int16_t *raw_a, *raw_w, *raw_b;
extern int16_t raw_data_offset[9];
extern int16_t *raw_offset_a, *raw_offset_w, *raw_offset_b;
extern uint32_t raw_v; // voltage
extern float data[28];
extern float *a, *w, *b, *euler, *quaternion, *quaternion_acc, *quaternion_mag;
extern float offset_a[3], offset_w[3];
extern float cos_calib, quaternion_calib[4];

// Parameters for Kalman filter
extern float F[18]; // state-transition model
extern float H[18]; // 4x4 identity matrix
extern float K[18]; // optimal Kalman gain
extern float P[18]; // estimate covariance
extern float Pp[18]; // a priori state estimate covariance
extern float Q[18]; // process noise covariance
extern float R[18]; // observation noise covariance
extern float M3x3buffer[11];
extern float M4x1buffer1[6];
extern float M4x1buffer2[6];
extern float M4x4buffer1[18];
extern float M4x4buffer2[18];
extern float M4x4buffer3[18];
extern float x[6]; // state
extern float xp[6]; // a priori state estimate
extern float z[6]; // quaternion
extern float eulerK[3], quaternionK[4];
extern float I_hat[5];
extern float J_hat[5];
extern float K_hat[5];

// Parameters for PID
#define mass_center				0.950	// kg
#define mass_motor				0.24	// kg
#define radius_center			0.05	// m
#define arm_length				0.225	// m
// jx, jy = 0.4*mass_center*radius_center^2 + 2*mass_motor*arm_length^2
// jz     = 0.4*mass_center*radius_center^2 + 4*mass_motor*arm_length^2
#define jx						0.02525
#define jy						0.02525
#define jz						0.04955
extern float euler_set[3];
extern float yaw_set_delta;
extern float quaternion_set[4];
extern float quaternion_delta[4];
extern float angle_delta;
extern float prev_angle_delta;
extern float w_req[3];
extern float torque_req[3];
extern float thrust_delta[4];
extern float thrust_base; // base thrust (in gf) of each rotor
extern float thrust[4];
extern float p1;
extern float d1;
extern float p2;
extern float l2;

// Parameters for command handler
extern volatile int XBEEIntflag;
#define XBEEIntmsg_size	100
extern volatile int XBEEIntmsg_pointer;
extern volatile char XBEEIntmsg[XBEEIntmsg_size];
extern int STATUS_startup;
extern int STATUS_loop;
extern int STATUS_stoploop;
extern int STATUS_CTRL_h;
extern int STATUS_CTRL_v;
extern int STATUS_CTRL_p;
extern int STATUS_CTRL_X;
extern int STATUS_CTRL_w;
extern int STATUS_CTRL_s;
extern int STATUS_CTRL_ad;
extern int STATUS_CTRL_58;
extern int STATUS_CTRL_46;
