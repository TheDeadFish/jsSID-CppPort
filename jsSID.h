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

typedef struct { unsigned char regs[7]; unsigned char ADSRstate, envcnt,
	expcnt, muteMask, noiseCache; unsigned ratecnt, prevwfout, 
	phaseaccu, noise_LFSR; unsigned short period; } SidChnl;
typedef struct { unsigned char regs[4], model_6581, sourceMSBrise;
	unsigned sourceMSB, rand_state; float prevlowpass, prevbandpass, cutoff,
	resonance, cutoff_ratio, cutoff_bias; SidChnl chnl[3]; } SidChip;

void jsSID_init(SidChip* This, int samplerate, int model);
void jsSID_write(SidChip* This, int addr, int data);
float jsSID_render(SidChip* This);
void jsSID_setMuteMask(SidChip* This, int mute_mask);

#define JSSID_SCALEDOWN (1.0F/0x30000000)
#define JSSID_SCALE(o,s) (o*s*JSSID_SCALEDOWN)

#ifdef __cplusplus
}
#endif

#endif
