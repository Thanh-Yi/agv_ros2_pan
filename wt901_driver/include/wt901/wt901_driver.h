#ifndef WT901_DRIVER_H
#define WT901_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

int  wt901_init(const char *dev, int baud);
void wt901_step(void);
//int  wt901_set_zero(void);
int  wt901_get_angle(float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif

#endif

