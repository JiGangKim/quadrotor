/*
 * mymatrix.h - vector/matrix computation
 *
 *  Created on: 2016. 7. 18.
 *      Author: jgkim2020
 *
 * I do not take any responsibility for any consequences resulting from the
 * use of this software. USE AT YOUR OWN RISK!
 *
 */

float dotProduct(float *v1, float *v2);
void crossProduct(float *v1, float *v2, float *v);
void normalize(float *v1, float *v);
void IdentityMatrix(float *M);
void ScalarXMatrix(float scalar, float *M1, float *M);
void Matrixaddition(float *M1, float *M2, float *M);
void Matrixmultiplication(float *M1, float *M2, float *M);
float DetMatrix(float *M);
void InvMatrix(float *M, float *InvM, float *Mbuffer);
void transMatrix(float *M, float *transM);
