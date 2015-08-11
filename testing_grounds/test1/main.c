#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

void acrobotDynamics(double* x, double u, double* xd);
void matrixInverse(double M[2][2], double invM[2][2]);
void matrixMultiply(double A[2][2], double B[2], double C[2]);

int main(void) {

    double xn[4] = {-2.1890,6.3523,1.7173,-2.8063};
    //double xn[4] = {0,0,0,0};
    double k1[4];
    acrobotDynamics(xn,-20,k1);

    printf("k1 = [%f,%f,%f,%f\n]", k1[0],k1[1],k1[2],k1[3]);

    return 0;
}

void acrobotDynamics(double* x, double u, double* xd)
{
    // acrobot parameters
    int m1 = 1;
    int m2 = 1;
    double l1 = 1;
    double l2 = 1;
    double lc1 = l1/2;
    double lc2 = l2/2;
    double Ic1 = (lc1*lc1)/3;
    double Ic2 = (lc2*lc2)/3;
    double I1 = Ic1+m1*lc1*lc1;
    double I2 = Ic2+m2*lc2*lc2;
    double b1 = 0.4;
    double b2 = 0.4;
    double g = 9.8;

    printf("x[2] = %f\n",x[2]);

    double H[2][2] = {{I1 + I2 + m2*l1*l1 + 2*m2*l1*lc2*cos(x[2]),
                       I2 + m2*l1*lc2*cos(x[2])},
                      {I2 + m2*l1*lc2*cos(x[2]),
                       I2}};
    printf("H = [%f, %f]\n    [%f, %f]\n",H[0][0],H[0][1],H[1][0],H[1][1]);

    double C[2][2] = {{-2*m2*l1*lc2*sin(x[2])*x[3] + b1,
                       -m2*l1*lc2*sin(x[2])*x[3]},
                      {m2*l1*lc2*sin(x[2])*x[1],
                       b2}};

    printf("C = [%f, %f]\n    [%f, %f]\n",C[0][0],C[0][1],C[1][0],C[1][1]);

    double G[2] = {m1*g*lc1*sin(x[0]) + m2*g*(l1*sin(x[0])+lc2*sin(x[0]+x[2])),
                   m2*g*lc2*sin(x[0]+x[2])};

    printf("G = [%f, %f]",G[0],G[1]);

    double B[2] = {0,u};

    double invH[2][2];
    matrixInverse(H,invH);

    double C_qd[2];
    double qd[2] = {x[1],x[3]};
    matrixMultiply(C,qd,C_qd);

    double temp[2];
    temp[0] = B[0] - C_qd[0] - G[0];
    temp[1] = B[1] - C_qd[1] - G[1];

    double qdd[2];
    matrixMultiply(invH,temp,qdd);

    xd[0] = x[1];
    xd[1] = qdd[0];
    xd[2] = x[3];
    xd[3] = qdd[1];
}

void matrixInverse(double M[2][2], double invM[2][2])
{
    double detInv = 1/(M[0][0]*M[1][1] - M[0][1]*M[1][0]);

    invM[0][0] = detInv*M[1][1];
    invM[0][1] = detInv*M[0][1]*-1;
    invM[1][0] = detInv*M[1][0]*-1;
    invM[1][1] = detInv*M[0][0];
}

void matrixMultiply(double A[2][2], double B[2], double C[2])
{
    C[0] = A[0][0]*B[0] + A[0][1]*B[1];
    C[1] = A[1][0]*B[0] + A[1][1]*B[1];
}
