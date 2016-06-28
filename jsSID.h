#ifndef _jsSID_H_
#define _jsSID_H_

#ifdef __cplusplus
extern "C" {
#endif

void jsSID_init(int sampleRate, int model);
void jsSID_write(int chip, int addr, int data);
float jsSID_getVolume(int channel);
void jsSID_setMuteMask(int mute_mask);
double jsSID_render(int chip);

#ifdef __cplusplus
}
#endif

#endif
