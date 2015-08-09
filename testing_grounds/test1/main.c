#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

void matrixInverse(double M[2][2], double invM[2][2]);

int main(void) {
    double A[2][2] = {{1,2},{3,4}};
    double B[2][2];

    matrixInverse(A,B);

    printf("%f %f\n%f %f\n", A[0][0],A[0][1],A[1][0],A[1][1]);
    printf("%f %f\n%f %f\n", B[0][0],B[0][1],B[1][0],B[1][1]);

    return 0;
}

void matrixInverse(double M[2][2], double invM[2][2])
{
    double detInv = 1/(M[0][0]*M[1][1] - M[0][1]*M[1][0]);

    invM[0][0] = detInv*M[1][1];
    invM[0][1] = detInv*M[0][1]*-1;
    invM[1][0] = detInv*M[1][0]*-1;
    invM[1][1] = detInv*M[0][0];
}
