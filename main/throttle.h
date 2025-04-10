#ifndef __THROTTLE_H__
#define __THROTTLE_H__


extern void throttle_setup(float limit, int stepsPerC);
extern int throttle_check(float temperature, int tune);

#endif