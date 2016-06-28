/*
		DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
                    Version 2, December 2004

 Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>

 Everyone is permitted to copy and distribute verbatim or modified
 copies of this license document, and changing it is allowed as long
 as the name is changed.

            DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
   TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

  0. You just DO WHAT THE FUCK YOU WANT TO.
*/

// jsSID by Hermit (Mihaly Horvath), (Year 2016) http://hermit.sidrip.com
// Converted to C/C++ by DeadFish Shitware

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
