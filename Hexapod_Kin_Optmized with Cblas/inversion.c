
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <cblas.h>
#include <lapacke.h>

//Matrix Inversion Function
float Inv(float *A, unsigned n)
{
    int ipiv[n];
    float ret;

    ret =  LAPACKE_sgetrf(LAPACK_ROW_MAJOR, n, n, A, n, ipiv);

    if (ret !=0)
        return ret;

    ret = LAPACKE_sgetri(LAPACK_ROW_MAJOR, n, A, n, ipiv);
    return ret;
}


//Test:
void main()
{

//Row Major
float A[9] = {1,2,2,2,2,2,2,2,1};

Inv(A,3);

int i = 0;

for(i=0; i<9; i++)
  printf("%f\n", A[i]);

}
