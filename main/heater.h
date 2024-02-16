#ifndef __HEATER__
#define __HEATER__

void heater_init(int fullRoundMs, int levels);
void heater_close(void);
void heater_setlevel(int level);
int heater_getlevel(void);

#endif