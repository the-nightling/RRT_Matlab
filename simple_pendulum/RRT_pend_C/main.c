/*
 * Computes the sequence of control actions needed for the swing up of
 * a simple pendulum using a Rapidly Exploring Random Tree.
 * The tree will grow from the initial state and span the phase space area,
 * looking for the goal point/state.
 *
 * Author: Nirav Domah
 * Date: 08/08/15
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>

double generateRandomDouble(double min, double max);
void euclidianDistSquare(double * A, double B[][2], int lengthOfB, double* listOfDistSq);
int findMin(double array[], int lengthOfArray);
void pendulumDynamics(double* x, double u, double* xd);

int main(void)
{
    double x0[] = {-M_PI/2,0};   // initial state; angle position measured from x-axis
    double xG[] = {M_PI/2,0};    // goal state

    double xlimits[2][2] = {{-M_PI,M_PI},{-10,10}};  // state limits; angular position between -pi & pi rad; angular velocity between -10 & 10 rad/s

    // control torques to be used: linspace(-5,5,20)
    double U[] = {-5.0000,-4.4737,-3.9474,-3.4211,-2.8947,-2.3684,-1.8421,-1.3158,-0.7895,-0.2632,
                5.0000, 4.4737, 3.9474, 3.4211, 2.8947, 2.3684, 1.8421, 1.3158, 0.7895, 0.2632};
    int lengthOfU = (int)( sizeof(U)/sizeof(U[0]) );

    double dt = 0.01;            // time interval between application of subsequent control torques

    int N = 50000;      // max number of iterations (if this value is edited, modify initialization of G as well)

    // static memory allocation
    double xn[2];        // stores a state
    double xd[2];
    double G[50000][2] = { [0 ... 50000-1] = {-M_PI/2,0} }; // graph of states in RRT; each index corresponds to a vertex (using designated initializer)
    int P[N];         // stores index of parent state for each state in graph G
    int Ui[N];        // stores index of control actions in U (each state will use a control action value in U)
    double u_path[1000]; // stores sequence of control actions (solution to problem)
    int xbi = 0;    // stores sequence of states joining initial to goal state
    double xn_c[lengthOfU][2]; // stores temporary achievable states from a particular vertex

    double dsq[N];  // stores distance square values

    srand(time(NULL));  // initialize random number generator


    // keep growing RRT until goal found or run out of iterations
    int n;
    for(n = 1; n < N; n++)
    {
        // get random state
        xn[0] = generateRandomDouble(xlimits[0][0],xlimits[0][1]);
        xn[1] = generateRandomDouble(xlimits[1][0],xlimits[1][1]);

        // find distances between that state point and every vertex in RRT
        euclidianDistSquare(xn,G,n,dsq);

        // select RRT vertex closest to the state point
        int minIndex = findMin(dsq,n);

        // from the closest RRT vertex, compute all the states that can be reached,
        // given the pendulum dynamics and available torques
        int ui;
        for(ui = 0; ui < lengthOfU; ui++)
        {
            pendulumDynamics(G[minIndex],U[ui],xd);
            xn_c[ui][0] = G[minIndex][0] + dt*xd[0];
            xn_c[ui][1] = G[minIndex][1] + dt*xd[1];
        }

        // select the closest reachable state point
        euclidianDistSquare(xn,xn_c,lengthOfU,dsq);
        ui = findMin(dsq,lengthOfU);
        xn[0] = xn_c[ui][0];
        xn[1] = xn_c[ui][1];

        // if angular position is greater than pi rads, wrap around
        if(xn[0] > M_PI || xn[0] < -M_PI)
            xn[0] = fmod((xn[0]+M_PI), (2*M_PI)) - M_PI;

        // link reachable state point to the nearest vertex in the tree
        G[n][0] = xn[0];
        G[n][1] = xn[1];
        P[n] = minIndex;
        Ui[n] = ui;

        if( (xn[0] <= xG[0]+0.1) && (xn[0] >= xG[0]-0.1) )
        {
            if( (xn[1] <= xG[1]+0.5) && (xn[1] >= xG[1]-0.5) )
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
void euclidianDistSquare(double* A, double B[][2], int lengthOfB, double* listOfDistSq)
{
    int i;
    for(i = 0; i < lengthOfB; i++)
        listOfDistSq[i] = pow((B[i][0] - A[0]),2) + pow((B[i][1] - A[1]),2);
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
 * Computes x_dot of the pendulum, given x and a control input u
 */
void pendulumDynamics(double* x, double u, double* xd)
{
    // pendulum parameters
    int m = 1;                  // mass
    int l = 1;                  // length of pendulum link
    int I = m*l*l;              // moment of inertia
    double g = 9.8;              // acceleration due to gravity
    double b = 0.1;              // damping factor

    xd[0] = x[1];
    xd[1] = (u - m*g*l*sin((M_PI/2)-x[0]) - b*x[1]) / I;
}
