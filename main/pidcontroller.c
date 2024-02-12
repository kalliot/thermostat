#include <time.h>
#include <stdio.h>
#include <math.h>
#include "pidcontroller.h"


float target = 20.0;
float startDiff = 2.0;
int maxTune = 1;
int checkInterval = 60;
float prevValue = 0.0;
float tempDiverge = 0.05;
float speedDiverge = 0.1;
time_t prevCheck;
int tuneValue = 0;



// pidcontroller_init(10, 10, 2.0, 0.05, 0.1)
void pidcontroller_init(int interval, int max, float diff, float tDiverge, float sDiverge)
{
    time(&prevCheck);
    checkInterval   = interval;
    startDiff       = diff;
    maxTune         = max;
    tempDiverge     = tDiverge;
    speedDiverge    = sDiverge;
    return;
}

void pidcontroller_target(float newTarget)
{
    if (newTarget != target)
    {
        target = newTarget;

        if (prevValue == 0.0) return;

        float diff = target - prevValue;
        if (diff > startDiff)         
        {
            tuneValue = maxTune-1;
        }
        else if (diff > (startDiff / 2))    
        {
            tuneValue = maxTune / 2;
        }
        else
            tuneValue = 0;
    }    
    return;
}


int pidcontroller_tune(float pv)
{
    time_t now;
    float speed = 0.001;

    time(&now);

    float diff = target - pv;
    if (prevValue == 0.0) // at startup
    {
        if (diff > startDiff)         
        {
            tuneValue = maxTune-1;
        }
        else if (diff > (startDiff / 2))    
        {
            tuneValue = maxTune / 2;    
        }
        prevCheck = now;
        prevValue = pv;
        return tuneValue;
    }    

    if ((now - prevCheck) < checkInterval)
        return tuneValue;

    speed = 60 * (pv - prevValue) / checkInterval;
    prevValue = pv;
    prevCheck = now;

    if (fabs(diff) < tempDiverge) 
    {
        if (speed >= speedDiverge) 
        {
            tuneValue--;
        }    
        else if (speed <= (speedDiverge * -1.0))  
        { 
            tuneValue++;
        }    
        else
        {
            return tuneValue;
        }    
    }    
    else {
        if (diff > 0.0) // under target
        {
            if (speed <= speedDiverge) // speed diverge
            {  
                tuneValue++;
            }    
        }
        if (diff < 0.0) // over target
        {
            if (speed >= speedDiverge * -1.0)   // speed diverge
            {
                tuneValue--;
            }       
        }
    }
    if (tuneValue >= maxTune)
        tuneValue = maxTune-1;
    if (tuneValue < 0)
        tuneValue = 0;
    return tuneValue;
}