#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

int findMin(float array[], int lengthOfArray);

int main(void) {
    int N = 20000;

    double G[20000][2] = { [0 ... 20000-1] = {-M_PI/2,1} };

    int i;
    for(i=0; i < N; i++)
    {
        printf("%f %f \n",G[i][0],G[i][1]);
    }

    return 0;
}

