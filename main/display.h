#ifndef __DISPLAY__
#define __DISPLAY__

int display_init(void);
void display_show(int f1, float f2);
void display_close(void);
void display_text(char *t);
void display_clear(void);
void display_brightness(int b);


#endif