#ifndef __PIDCONTROLLER__
#define __PIDCONTROLLER__

void pidcontroller_init(int interval, int max, float diff, float tDiverge, float sDiverge);
void pidcontroller_target(float newTarget);
int pidcontroller_tune(float pv);

#endif 