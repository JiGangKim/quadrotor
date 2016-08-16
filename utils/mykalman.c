/*
 * mykalman.c - kalman filter
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
#include <math.h>
#include "myvariable.h"
#include "mymatrix.h"

void ACCMAGtoTBXYZ(float *a, float *b, float *euler, int a_enable, int b_enable)
{	// get Tait-Bryan XYZ angle from accelerometer data and magnetometer data
	if(a_enable != 0)
	{
		K_hat[2] = a[0]/a[3]; // center of Earth (body frame)
		K_hat[3] = a[1]/a[3];
		K_hat[4] = a[2]/a[3];
		euler[0] = atan2f(K_hat[3], K_hat[4]); // roll
		euler[1] = asinf(-K_hat[2]); // pitch
		euler[2] = 0; // yaw
	}
	if(b_enable != 0)
	{
		I_hat[2] = b[0]/b[3]; // North (body frame)
		I_hat[3] = b[1]/b[3];
		I_hat[4] = b[2]/b[3];
		crossProduct(K_hat, I_hat, J_hat); // East (body frame)
		J_hat[2] = J_hat[2]/sqrtf(dotProduct(J_hat, J_hat));
		J_hat[3] = J_hat[3]/sqrtf(dotProduct(J_hat, J_hat));
		J_hat[4] = J_hat[4]/sqrtf(dotProduct(J_hat, J_hat));
		euler[2] = atan2f(J_hat[2], I_hat[2]); // yaw
	}
}

void ACCMAGtoQUATERNION(float *a, float *b, float *quaternion)
{
	K_hat[2] = a[0]/a[3];
	K_hat[3] = a[1]/a[3];
	K_hat[4] = a[2]/a[3];
	if(K_hat[4] >= 0.0)
	{
		quaternion_acc[0] = sqrtf(0.5 + 0.5*K_hat[4]);
		quaternion_acc[1] = K_hat[3]/sqrtf(2 + 2*K_hat[4]);
		quaternion_acc[2] = -K_hat[2]/sqrtf(2 + 2*K_hat[4]);
		quaternion_acc[3] = 0;
	}
	else
	{
		quaternion_acc[0] = K_hat[3]/sqrtf(2 - 2*K_hat[4]);
		quaternion_acc[1] = sqrtf(0.5 - 0.5*K_hat[4]);
		quaternion_acc[2] = 0;
		quaternion_acc[3] = K_hat[2]/sqrtf(2 - 2*K_hat[4]);
	}
	// q_acc*m
	quaternion_mag[0] = -quaternion_acc[1]*b[0] - quaternion_acc[2]*b[1] - quaternion_acc[3]*b[2];
	quaternion_mag[1] =  quaternion_acc[0]*b[0] - quaternion_acc[3]*b[1] + quaternion_acc[2]*b[2];
	quaternion_mag[2] =  quaternion_acc[3]*b[0] + quaternion_acc[0]*b[1] - quaternion_acc[1]*b[2];
	quaternion_mag[3] = -quaternion_acc[2]*b[0] + quaternion_acc[1]*b[1] + quaternion_acc[0]*b[2];
	// L = q_acc*m*q_acc'
	I_hat[2] = quaternion_mag[1]*quaternion_acc[0] - quaternion_mag[0]*quaternion_acc[1] + quaternion_mag[3]*quaternion_acc[2] - quaternion_mag[2]*quaternion_acc[3];
	I_hat[3] = quaternion_mag[2]*quaternion_acc[0] - quaternion_mag[3]*quaternion_acc[1] - quaternion_mag[0]*quaternion_acc[2] + quaternion_mag[1]*quaternion_acc[3];
	I_hat[4] = quaternion_mag[3]*quaternion_acc[0] + quaternion_mag[2]*quaternion_acc[1] - quaternion_mag[1]*quaternion_acc[2] - quaternion_mag[0]*quaternion_acc[3];
	euler[2] = atan2f(I_hat[3], I_hat[2]);
	quaternion_mag[0] = cosf(0.5*euler[2]);
	quaternion_mag[1] = 0;
	quaternion_mag[2] = 0;
	quaternion_mag[3] = -sinf(0.5*euler[2]);
	// q = q_mag*q_acc
	quaternion[0] = quaternion_mag[0]*quaternion_acc[0] - quaternion_mag[1]*quaternion_acc[1] - quaternion_mag[2]*quaternion_acc[2] - quaternion_mag[3]*quaternion_acc[3];
	quaternion[1] = quaternion_mag[1]*quaternion_acc[0] + quaternion_mag[0]*quaternion_acc[1] - quaternion_mag[3]*quaternion_acc[2] + quaternion_mag[2]*quaternion_acc[3];
	quaternion[2] = quaternion_mag[2]*quaternion_acc[0] + quaternion_mag[3]*quaternion_acc[1] + quaternion_mag[0]*quaternion_acc[2] - quaternion_mag[1]*quaternion_acc[3];
	quaternion[3] = quaternion_mag[3]*quaternion_acc[0] - quaternion_mag[2]*quaternion_acc[1] + quaternion_mag[1]*quaternion_acc[2] + quaternion_mag[0]*quaternion_acc[3];
}

void TBXYZtoQUATERNION(float *euler, float *quaternion)
{	// Convert Tait-Bryan XYZ angles to quaternion(representative of rotation from body frame to vehicle frame)
	float sinRoll, cosRoll, sinPitch, cosPitch, sinYaw, cosYaw;
	sinRoll = sinf(euler[0]/2);
	cosRoll = cosf(euler[0]/2);
	sinPitch = sinf(euler[1]/2);
	cosPitch = cosf(euler[1]/2);
	sinYaw = sinf(euler[2]/2);
	cosYaw = cosf(euler[2]/2);
	quaternion[0] = cosRoll*cosPitch*cosYaw + sinRoll*sinPitch*sinYaw;
	quaternion[1] = sinRoll*cosPitch*cosYaw - cosRoll*sinPitch*sinYaw;
	quaternion[2] = cosRoll*sinPitch*cosYaw + sinRoll*cosPitch*sinYaw;
	quaternion[3] = cosRoll*cosPitch*sinYaw - sinRoll*sinPitch*cosYaw;
}

void QUATERNIONtoTBXYZ(float *euler, float *quaternion)
{	// Convert quaternion(representative of rotation from body frame to vehicle frame) to Tait-Bryan XYZ angles
	euler[0] = atan2f(2*quaternion[2]*quaternion[3] + 2*quaternion[0]*quaternion[1],
					quaternion[0]*quaternion[0] - quaternion[1]*quaternion[1] - quaternion[2]*quaternion[2] + quaternion[3]*quaternion[3]);
	euler[1] = asinf(-2*quaternion[1]*quaternion[3] + 2*quaternion[0]*quaternion[2]);
	euler[2] = atan2f(2*quaternion[1]*quaternion[2] + 2*quaternion[0]*quaternion[3],
					quaternion[0]*quaternion[0] + quaternion[1]*quaternion[1] - quaternion[2]*quaternion[2] - quaternion[3]*quaternion[3]);
}

void KALMAN_ACCMAGGYRO()
{
	// F (state-transition model)
	F[0*4+0+2] = 1 + dt*0.5*0;		F[0*4+1+2] = 0 - dt*0.5*w[0];	F[0*4+2+2] = 0 - dt*0.5*w[1];	F[0*4+3+2] = 0 - dt*0.5*w[2];
	F[1*4+0+2] = 0 + dt*0.5*w[0];	F[1*4+1+2] = 1 + dt*0.5*0;		F[1*4+2+2] = 0 + dt*0.5*w[2];	F[1*4+3+2] = 0 - dt*0.5*w[1];
	F[2*4+0+2] = 0 + dt*0.5*w[1];	F[2*4+1+2] = 0 - dt*0.5*w[2];	F[2*4+2+2] = 1 + dt*0.5*0;		F[2*4+3+2] = 0 + dt*0.5*w[0];
	F[3*4+0+2] = 0 + dt*0.5*w[2];	F[3*4+1+2] = 0 + dt*0.5*w[1];	F[3*4+2+2] = 0 - dt*0.5*w[0];	F[3*4+3+2] = 1 + dt*0.5*0;

	// Predict (xp = F*x, Pp = F*P*F' + Q)
	Matrixmultiplication(F, x, xp); // xp = F*x
	normalize(xp, xp);
	Matrixmultiplication(F, P, M4x4buffer1); // buffer1 = F*P
	transMatrix(F, M4x4buffer2); // buffer2 = F'
	Matrixmultiplication(M4x4buffer1, M4x4buffer2, M4x4buffer3); // buffer3 = buffer1*buffer2
	Matrixaddition(M4x4buffer3, Q, Pp); // Pp = buffer3(F*P*F') + Q

	// Kalman Gain (K = Pp*H'*inv(H*Pp*H' + R), H = I)
	Matrixaddition(Pp, R, M4x4buffer1); // buffer1 = Pp + R
	InvMatrix(M4x4buffer1, M4x4buffer2, M3x3buffer); // buffer2 = inv(buffer1)
	Matrixmultiplication(Pp, M4x4buffer2, K); // K = Pp*buffer2(inv(Pp + R))

	// Measure
	ACCMAGtoQUATERNION(a, b, quaternion);
	z[2] = quaternion[0];
	z[3] = quaternion[1];
	z[4] = quaternion[2];
	z[5] = quaternion[3];
	if(dotProduct(z, x) < 0) ScalarXMatrix(-1, z, z); // remove discontinuity

	// Update (x = xp + K*(z - H*xp), P = Pp - K*H*Pp, H = I)
	ScalarXMatrix(-1, xp, M4x1buffer1); // buffer1 = -xp
	Matrixaddition(z, M4x1buffer1, M4x1buffer2); // buffer2 = z - xp
	Matrixmultiplication(K, M4x1buffer2, M4x1buffer1); // buffer1 = K*buffer2
	Matrixaddition(xp, M4x1buffer1, x); // x = xp + buffer1(K*(z - xp))
	normalize(x, x);
	Matrixmultiplication(K, Pp, M4x4buffer1); // buffer1 = K*Pp
	ScalarXMatrix(-1, M4x4buffer1, M4x4buffer1); // buffer1 = -buffer1
	Matrixaddition(Pp, M4x4buffer1, P); // P = Pp + buffer1(-K*Pp)

	// Kalman Filtered Output
	quaternionK[0] = x[2];
	quaternionK[1] = x[3];
	quaternionK[2] = x[4];
	quaternionK[3] = x[5];
	QUATERNIONtoTBXYZ(eulerK, quaternionK); // quaternionK to eulerK
}
