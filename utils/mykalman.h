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

void ACCMAGtoTBXYZ(float *a, float *b, float *euler, int a_enable, int b_enable);
void ACCMAGtoQUATERNION(float *a, float *b, float *quaternion);
void TBXYZtoQUATERNION(float *euler, float *quaternion);
void QUATERNIONtoTBXYZ(float *euler, float *quaternion);
void KALMAN_ACCMAGGYRO();
