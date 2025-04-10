#include <stdio.h>
#include <stdlib.h>

static int floatC = 30;
static int stepsPerC = 10;

void throttle_setup(float limit, int stepsPerC)
{
    limitC = limit;
    stepsPerC = stepsPerC;
}   

// decrease tune if temperature is above limit
// return new tune value
int throttle_check(float temperature, int tune)
{
    float diff;
    int ret = tune;

    diff = temperature - limitC;
    if (diff > 0)
    {
        ret = tune - int(diff * stepsPerC);
    }
    return ret;
}
