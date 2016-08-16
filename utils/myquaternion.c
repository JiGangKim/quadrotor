/*
 * myquaternion.c - quaternion operation
 *
 *  Created on: 2016. 7. 18.
 *      Author: jgkim2020
 *
 * I do not take any responsibility for any consequences resulting from the
 * use of this software. USE AT YOUR OWN RISK!
 *
 */

#include "myvariable.h"
#include "mymatrix.h"

void Quaternionmultiplication(float *q1, float *q2, float *q)
{
	q[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
	I_hat[2] = q1[1]; J_hat[2] = q2[1];
	I_hat[3] = q1[2]; J_hat[3] = q2[2];
	I_hat[4] = q1[3]; J_hat[4] = q2[3];
	crossProduct(I_hat, J_hat, K_hat);
	q[1] = K_hat[2] + q2[0]*q1[1] + q1[0]*q2[1];
	q[2] = K_hat[3] + q2[0]*q1[2] + q1[0]*q2[2];
	q[3] = K_hat[4] + q2[0]*q1[3] + q1[0]*q2[3];
}

void Quaternion_qvq(float *q, float *v)
{
	float vimsi[3];
	vimsi[0] = (1 - 2*(q[2]*q[2] + q[3]*q[3]))*v[0] + 2*(q[1]*q[2] - q[0]*q[3])*v[1] + 2*(q[1]*q[3] + q[0]*q[2])*v[2];
	vimsi[1] = 2*(q[1]*q[2] + q[0]*q[3])*v[0] + (1 - 2*(q[1]*q[1] + q[3]*q[3]))*v[1] + 2*(q[2]*q[3] - q[0]*q[1])*v[2];
	vimsi[2] = 2*(q[1]*q[3] - q[0]*q[2])*v[0] + 2*(q[2]*q[3] + q[0]*q[1])*v[1] + (1 - 2*(q[1]*q[1] + q[2]*q[2]))*v[2];
	v[0] = vimsi[0];
	v[1] = vimsi[1];
	v[2] = vimsi[2];
}
