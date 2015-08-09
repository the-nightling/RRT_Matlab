/*
 * Computes the sequence of control actions needed for the swing up of
 * an acrobot using a Rapidly Exploring Random Tree.
 * The tree will grow from the initial state and span the phase space area,
 * looking for the goal point/state.
 *
 * Author: Nirav Domah
 * Date: 09/08/15
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

double generateRandomDouble(double min, double max);
void euclidianDistSquare(double * A, double B[][4], int lengthOfB, double* listOfDistSq);
int findMin(double array[], int lengthOfArray);
void acrobotDynamics(double* x, double u, double* xd);
void matrixInverse(double M[2][2], double invM[2][2]);
void matrixMultiply(double A[2][2], double B[2], double C[2]);

int main(void)
{
    double x0[] = {0,0,0,0};   // initial state
    double xG[] = {M_PI,0,0,0};    // goal state

    double xlimits[4][2] = {{-M_PI,M_PI},{-10,10},{-M_PI,M_PI},{-10,10}};  // state limits; angular position between -pi & pi rad; angular velocity between -10 & 10 rad/s

    // control torques to be used: linspace(-20,20,5)
    double U[] = {-20,-10,0,10,20};
    int lengthOfU = (int)( sizeof(U)/sizeof(U[0]) );

    double dt = 0.04;            // time interval between application of subsequent control torques

    int N = 100000;      // max number of iterations (if this value is edited, modify initialization of G as well)

    // static memory allocation
    double xn[4];        // stores a state
    double k1[4],k2[4],k3[4],k4[4],kTemp[4];
    double G[100000][4] = { [0 ... 100000-1] = {0,0,0,0} }; // graph of states in RRT; each index corresponds to a vertex (using designated initializer)
    int P[N];         // stores index of parent state for each state in graph G
    int Ui[N];        // stores index of control actions in U (each state will use a control action value in U)
    double u_path[10000]; // stores sequence of control actions (solution to problem)
    int xbi = 0;    // stores sequence of states joining initial to goal state
    double xn_c[lengthOfU][4]; // stores temporary achievable states from a particular vertex

    double dsq[N];  // stores distance square values

    srand(time(NULL));  // initialize random number generator


    // keep growing RRT until goal found or run out of iterations
    int n;
    for(n = 1; n < N; n++)
    {
        // get random state
        xn[0] = generateRandomDouble(xlimits[0][0],xlimits[0][1]);
        xn[1] = generateRandomDouble(xlimits[1][0],xlimits[1][1]);
        xn[2] = generateRandomDouble(xlimits[2][0],xlimits[2][1]);
        xn[3] = generateRandomDouble(xlimits[3][0],xlimits[3][1]);

        // find distances between that state point and every vertex in RRT
        euclidianDistSquare(xn,G,n,dsq);

        // select RRT vertex closest to the state point
        int minIndex = findMin(dsq,n);

        // from the closest RRT vertex, compute all the states that can be reached,
        // given the pendulum dynamics and available torques
        int ui;
        for(ui = 0; ui < lengthOfU; ui++)
        {
            // using RK4 for dynamics
            acrobotDynamics(G[minIndex],U[ui],k1);

            kTemp[0] = G[minIndex][0]+0.5*k1[0]*dt;
            kTemp[1] = G[minIndex][1]+0.5*k1[1]*dt;
            kTemp[2] = G[minIndex][2]+0.5*k1[2]*dt;
            kTemp[3] = G[minIndex][3]+0.5*k1[3]*dt;
            acrobotDynamics(kTemp,U[ui],k2);

            kTemp[0] = G[minIndex][0]+0.5*k2[0]*dt;
            kTemp[1] = G[minIndex][1]+0.5*k2[1]*dt;
            kTemp[2] = G[minIndex][2]+0.5*k2[2]*dt;
            kTemp[3] = G[minIndex][3]+0.5*k2[3]*dt;
            acrobotDynamics(kTemp,U[ui],k3);

            kTemp[0] = G[minIndex][0]+k3[0]*dt;
            kTemp[1] = G[minIndex][1]+k3[1]*dt;
            kTemp[2] = G[minIndex][2]+k3[2]*dt;
            kTemp[3] = G[minIndex][3]+k3[3]*dt;
            acrobotDynamics(kTemp,U[ui],k4);

            xn_c[ui][0] = G[minIndex][0] + dt*(1/6)*(k1[0]+2*k2[0]+2*k3[0]+k4[0]);
            xn_c[ui][1] = G[minIndex][1] + dt*(1/6)*(k1[1]+2*k2[1]+2*k3[1]+k4[1]);
            xn_c[ui][2] = G[minIndex][2] + dt*(1/6)*(k1[2]+2*k2[2]+2*k3[2]+k4[2]);
            xn_c[ui][3] = G[minIndex][3] + dt*(1/6)*(k1[3]+2*k2[3]+2*k3[3]+k4[3]);
        }

        // select the closest reachable state point
        euclidianDistSquare(xn,xn_c,lengthOfU,dsq);
        ui = findMin(dsq,lengthOfU);
        xn[0] = xn_c[ui][0];
        xn[1] = xn_c[ui][1];
        xn[2] = xn_c[ui][2];
        xn[3] = xn_c[ui][3];

        // if angular position is greater than pi rads, wrap around
        if(xn[0] > M_PI || xn[0] < -M_PI)
            xn[0] = fmod((xn[0]+M_PI), (2*M_PI)) - M_PI;

        if(xn[2] > M_PI || xn[2] < -M_PI)
            xn[2] = fmod((xn[2]+M_PI), (2*M_PI)) - M_PI;

        // link reachable state point to the nearest vertex in the tree
        G[n][0] = xn[0];
        G[n][1] = xn[1];
        G[n][2] = xn[2];
        G[n][3] = xn[3];
        P[n] = minIndex;
        Ui[n] = ui;

        if( (xn[0] > 2) || (xn[0] < -2) )
        {
            if( (xn[2] > -0.1) && (xn[2] < 0.1) )
            {
                break;
            }
        }

    }

    if(n == N)
    {
        printf("Simulation complete (goal not found; ran out of iterations)\n");
        printf("Number of iterations: %d\n",n);
    } else
    {
        printf("Simulation complete (goal found)\n");
        printf("Number of iterations: %d\n",n);

        xbi = n;
        int index = 0;
        while(xbi != 0)
        {
            u_path[index] = U[ Ui[xbi] ];
            index++;

            xbi = P[xbi];
        }

        FILE *dataFile = fopen("data.txt", "w");

        while(index > 0)
        {
            fprintf(dataFile, "%f\n",u_path[index-1]);
            index--;
        }

        fclose(dataFile);
    }


    return 0;
}


/*
 * generates a random double between the limits min and max
 */
double generateRandomDouble(double min, double max)
{
    return ( ((double)rand() / (double) RAND_MAX) * (max - min) ) + min;
}

/*
 * computes the euclidian distances squared from point A to every point in array B
 */
void euclidianDistSquare(double* A, double B[][4], int lengthOfB, double* listOfDistSq)
{
    int i;
    for(i = 0; i < lengthOfB; i++)
        listOfDistSq[i] = pow((B[i][0] - A[0]),2) + pow((B[i][1] - A[1]),2) + pow((B[i][2] - A[2]),2) + pow((B[i][3] - A[3]),2);
}

/*
 * finds the index of the minimum in an array
 */
int findMin(double array[], int lengthOfArray)
{
    int minIndex = 0;

    int i;
    for(i = 0; i < lengthOfArray; i++)
    {
        if(array[i] < array[minIndex])
            minIndex = i;
    }

    return minIndex;
}

/*
 * Computes x_dot of the acrobot, given x and a control input u
 */
void acrobotDynamics(double* x, double u, double* xd)
{
    // acrobot parameters
    int m1 = 1;
    int m2 = 1;
    int l1 = 1;
    int l2 = 1;
    double lc1 = 1;
    double lc2 = 1;
    double Ic1 = (lc1*lc1)/3;
    double Ic2 = (lc2*lc2)/3;
    double I1 = Ic1+m1*lc1*lc1;
    double I2 = Ic2+m2*lc2*lc2;
    double b1 = 0.4;
    double b2 = 0.4;
    double g = 9.8;

    double H[2][2] = {{I1 + I2 + m2*l1*l1 + 2*m2*l1*lc2*cos(x[2]),
                       I2 + m2*l1*lc2*cos(x[2])},
                      {I2 + m2*l1*lc2*cos(x[2]),
                       I2}};

    double C[2][2] = {{-2*m2*l1*lc2*sin(x[2])*x[3] + b1,
                       -m2*l1*lc2*sin(x[2])*x[3]},
                      {m2*l1*lc2*sin(x[2])*x[1],
                       b2}};

    double G[2] = {m1*g*lc1*sin(x[0]) + m2*g*(l1*sin(x[0])+lc2*sin(x[0]+x[2])),
                   m2*g*lc2*sin(x[0]+x[2])};

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
