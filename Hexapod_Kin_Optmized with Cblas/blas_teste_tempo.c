#include <stdio.h>
#include <time.h>
#include <math.h>
#include <cblas.h>
#include <lapacke.h>

//Program to Test the CBLAS MATRIX MULTIPLICATION

void main()
{

  /* C BLAS Matrix Multiplication Function
     //cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
            m, n, k, alpha, A, k, B, n, beta, C, n);

            A[m][k]  B[k][n] C[m][n]  alpha/beta = scale factor

            alpha*(A*B) + beta*C  = C
  */


  //Start Clock Timer:
  clock_t begin = clock();

  //Define the 6x6 Linear Regression Coeficient Matrix (Row Major)
  float RegCoef[36] = {1.88740566, -1.88740566, -1.78630476e-1, 1.70265653, -1.70265653, 1.7863047e-1,
                     -0.87999987, -0.87999987, 2.07628491, -1.19622617, -1.19622617, 2.07628491,
                      0.17310112, 0.17310112, 0.1730323, 0.17318892, 0.17318892, 0.1730323,
                      0.09379136, 0.09379136, 0.60417547, -0.69795643, -0.69795643, 0.60417547,
                      -7.47210598e-1, 7.47210598e-1, 4.53910209e-1, -2.92504249e-1, 2.92504249e-1, -4.53910209e-1,
                      -1.22550406, 1.22550406, -1.22207852, 1.22116327, -1.22116327, 1.22207852};

  float W0[6]= {-1.51694257e-9, -0.00958264, -11.31748409, -0.0012806, 6.14411133e-10, -3.11851055e-10};

  //Supposed Encoder Reading Vector
  float L[6] = {113,113,113,113,113,113};

  int m = 6;
  int n = 1;
  int k = 6;
  float alpha = 1.0;
  float beta = 1.0;

  cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
              m, n, k, alpha, RegCoef, k, L, n, beta, W0, n);

  //Clock to get time of execution END
  clock_t end = clock();

  //Time of execution calculation
  float time_spent = (float)(end - begin) / CLOCKS_PER_SEC;
  printf("%.15f\n", time_spent);


  // Sum and Multiplication Method:

  //Start Clock Timer:
  clock_t begin2 = clock();

  // Supposed Encoder Reading
  float L1 = 113; float L2 = 113; float L3 = 113;
  float L4 = 113; float L5 = 113; float L6 = 113;

  // Linear Regression Calculation
  float ForwardX = ((-1.51694257E-9) + L1*(1.88740566) + L2*(-1.88740566) + L3*(-1.78630476E-1) + L4*(1.70265653) + L5*(-1.70265653) + L6*(1.7863047E-1));
  float ForwardY = ((-0.00958264) + L1*(-0.87999987) + L2*(-0.87999987) + L3*(2.07628491) + L4*(-1.19622617) + L5*(-1.19622617) + L6*(2.07628491));
  float ForwardZ = ((-11.31748409) + L1*(0.17310112) + L2*(0.17310112) + L3*(0.1730323) + L4*(0.17318892) + L5*(0.17318892) + L6*(0.1730323));

  float ForwardRX = ((-0.0012806) + L1*(0.09379136) + L2*(0.09379136) + L3*(0.60417547) + L4*(-0.69795643) + L5*(-0.69795643) + L6*(0.60417547));
  float ForwardRY = ((6.14411133E-10) + L1*(-7.47210598E-1) + L2*(7.47210598E-1) + L3*(4.53910209E-1) + L4*(-2.92504249E-1) + L5*(2.92504249E-1) + L6*(-4.53910209E-1));
  float ForwardRZ = ((-3.11851055E-10) + L1*(-1.22550406) + L2*(1.22550406) + L3*(-1.22207852) + L4*(1.22116327) + L5*(-1.22116327) + L6*(1.22207852));

  //Define Initial Guess Matrix:
  float X2[6][1] = {0,0,0,0,0,0};

  X2[1][1] = ForwardX;   X2[2][1] = ForwardY;   X2[3][1] = ForwardZ;
  X2[4][1] = ForwardRX;  X2[5][1] = ForwardRY;  X2[6][1] = ForwardRZ;

  //Clock to get time of execution END
  clock_t end2 = clock();

  //Time of execution calculation
  float time_spent2 = (float)(end2 - begin2) / CLOCKS_PER_SEC;

  printf("%.15f\n", time_spent2);

}
