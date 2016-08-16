/*
 * mymatrix.c - vector/matrix computation
 *
 *  Created on: 2016. 7. 18.
 *      Author: jgkim2020
 *
 * I do not take any responsibility for any consequences resulting from the
 * use of this software. USE AT YOUR OWN RISK!
 *
 */

//*****************************************************************************
#include <math.h>
//*****************************************************************************

float dotProduct(float *v1, float *v2)
{
	int i;
	float sum;
	int size;
	sum = 0;
	size = v1[0];
	for(i = 2; i < size + 2; i++) sum = sum + v1[i]*v2[i];
	return sum;
}

void crossProduct(float *v1, float *v2, float *v)
{
	v[0] = 3, v[1] = 1;
	v[2] = v1[3]*v2[4] - v1[4]*v2[3];
	v[3] = v1[4]*v2[2] - v1[2]*v2[4];
	v[4] = v1[2]*v2[3] - v1[3]*v2[2];
}

void normalize(float *v1, float *v)
{
	int i;
	float length = 0;
	int size;
	length = sqrtf(dotProduct(v1, v1));
	size = v1[0];
	for(i = 2; i < size + 2; i++) v[i] = v1[i]/length;
}

void IdentityMatrix(float *M)
{
	int i, j, p;
	int size;
	size = M[0];
	for(i = 0; i < size; i++)
	{
		for(j = 0; j < size; j++)
		{
			p = i*size + j + 2;
			if(i == j) M[p] = 1;
			else M[p] = 0;
		}
	}
}

void ScalarXMatrix(float scalar, float *M1, float *M)
{
	int i, j;
	int row, column;
	int p;
	row = M1[0];
	column = M1[1];
	for(i = 0; i < row; i++)
	{
		for(j = 0; j < column; j++)
		{
			p = i*column + j + 2;
			M[p] = scalar*M1[p];
		}
	}
}

void Matrixaddition(float *M1, float *M2, float *M)
{
	int i, j;
	int row, column;
	int p;
	row = M1[0];
	column = M1[1];
	for(i = 0; i < row; i++)
	{
		for(j = 0; j < column; j++)
		{
			p = i*column + j + 2;
			M[p] = M1[p] + M2[p];
		}
	}
}

void Matrixmultiplication(float *M1, float *M2, float *M)
{
	int i, j, k;
	int m, n, o;
	int p, q, r;
	m = M1[0];
	n = M1[1];
	o = M2[1];
	for(i = 0; i < m; i++)
	{
		for(j = 0; j < o; j++)
		{
			p = i*o + j + 2;
			M[p] = 0;
			for(k = 0; k < n; k++)
			{
				q = i*n + k + 2;
				r = k*o + j + 2;
				M[p] = M[p] + M1[q]*M2[r];
			}
		}
	}
}

float DetMatrix(float *M) // crude
{
	float det, det00, det01, det02, det03;
	int size = M[0];

	if(size == 1) det = M[2];
	else if(size == 2) det = M[0*2+0+2]*M[1*2+1+2] - M[1*2+0+2]*M[0*2+1+2];
	else if(size == 3) det = M[0*3+0+2]*(M[1*3+1+2]*M[2*3+2+2] - M[2*3+1+2]*M[1*3+2+2]) + M[0*3+1+2]*(M[1*3+2+2]*M[2*3+0+2] - M[1*3+0+2]*M[2*3+2+2]) + M[0*3+2+2]*(M[1*3+0+2]*M[2*3+1+2] - M[2*3+0+2]*M[1*3+1+2]);
	else if(size == 4)
	{
		det00 = M[1*4+1+2]*(M[2*4+2+2]*M[3*4+3+2] - M[3*4+2+2]*M[2*4+3+2]) + M[1*4+2+2]*(M[2*4+3+2]*M[3*4+1+2] - M[2*4+1+2]*M[3*4+3+2]) + M[1*4+3+2]*(M[2*4+1+2]*M[3*4+2+2] - M[3*4+1+2]*M[2*4+2+2]);
        det01 = M[1*4+0+2]*(M[2*4+2+2]*M[3*4+3+2] - M[3*4+2+2]*M[2*4+3+2]) + M[1*4+2+2]*(M[2*4+3+2]*M[3*4+0+2] - M[2*4+0+2]*M[3*4+3+2]) + M[1*4+3+2]*(M[2*4+0+2]*M[3*4+2+2] - M[3*4+0+2]*M[2*4+2+2]);
        det02 = M[1*4+0+2]*(M[2*4+1+2]*M[3*4+3+2] - M[3*4+1+2]*M[2*4+3+2]) + M[1*4+1+2]*(M[2*4+3+2]*M[3*4+0+2] - M[2*4+0+2]*M[3*4+3+2]) + M[1*4+3+2]*(M[2*4+0+2]*M[3*4+1+2] - M[3*4+0+2]*M[2*4+1+2]);
        det03 = M[1*4+0+2]*(M[2*4+1+2]*M[3*4+2+2] - M[3*4+1+2]*M[2*4+2+2]) + M[1*4+1+2]*(M[2*4+2+2]*M[3*4+0+2] - M[2*4+0+2]*M[3*4+2+2]) + M[1*4+2+2]*(M[2*4+0+2]*M[3*4+1+2] - M[3*4+0+2]*M[2*4+1+2]);
        det = M[0*4+0+2]*det00 - M[0*4+1+2]*det01 + M[0*4+2+2]*det02 - M[0*4+3+2]*det03;
	}
	else det = 0;
	return det;
}
/*
float DetMatrix(float **M, int size) // recursive
{
	int i,j,j1,j2;
	float det = 0;
	float **A = NULL;

	if(size == 1) det = M[0][0];
	else if(size == 2) det = M[0][0]*M[1][1] - M[1][0]*M[0][1];
	else
    {
		A = (float **) malloc((size - 1)*sizeof(float *));
        for(i = 0;i < size-1; i++) A[i] = (float *) malloc((size - 1)*sizeof(float));

	    for(j1 = 0; j1 < size; j1++)
	    {
	    	for(i = 1; i < size; i++)
		    {
	    		j2 = 0;
                for(j = 0; j < size; j++)
			    {
                	if(j != j1)
                	{
                		A[i-1][j2] = M[i][j];
				        j2++;
                	}
			    }
		    }
	    	if(j1%2 == 0) det = det + M[0][j1]*DetMatrix(A, size-1);
	    	else det = det - M[0][j1]*DetMatrix(A, size-1);
	    }

	    for(i = 0; i < size-1; i++) free(A[i]);
	    free(A);
   }
   return(det);
}
*/
void InvMatrix(float *M, float *InvM, float *Mbuffer)
{
	int i, j, ii, jj, i1, j1;
    float det;
    int size;
    size = M[0];
    det = DetMatrix(M);

    for(j = 0; j < size; j++)
    {
    	for(i = 0; i < size; i++)
	    {
    		i1 = 0;
    		for(ii = 0; ii < size; ii++)
    		{
    			if(ii != i)
    			{
    				j1 = 0;
    				for(jj = 0; jj < size; jj++)
    				{
    					if(jj != j)
    					{
    						Mbuffer[i1*(size-1)+j1+2] = M[ii*size+jj+2];
    						j1++;
    					}
    				}
    				i1++;
    			}
    		}
    		if((i + j)%2 == 0) InvM[j*size+i+2] = DetMatrix(Mbuffer)/det;
    		else InvM[j*size+i+2] = -DetMatrix(Mbuffer)/det;
	    }
    }
}

void transMatrix(float *M, float *transM)
{
	int i, j;
	int row, column;
	row = M[0];
	column = M[1];

	for(i = 0; i < row; i++)
	{
		for(j = 0; j < column; j++)
		{
			transM[j*row+i+2] = M[i*column+j+2];
		}
	}
}
