// jsSID port by Hermit (Mihaly Horvath) to C, (Year 2016) http://hermit.sidrip.com
// Converted to C/C++ by DeadFish Shitware & Delek, cycle-exact version corrected/finalized by Hermit

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "jsSID.h"

typedef unsigned short u16;
typedef unsigned char byte;
typedef unsigned int u32;
typedef signed int s32;

#define READ_LIL16(p) *((u16*)p)
#define nothing() ({ asm(" "); })

enum { C64_PAL_CPUCLK = 985248,	SID_CHANNEL_AMOUNT = 3, 
	OUTPUT_SCALEDOWN = 0x1000000 * SID_CHANNEL_AMOUNT * 16};

enum { GATE_BITMASK=0x01, SYNC_BITMASK=0x02, RING_BITMASK=0x04,
	TEST_BITMASK=0x08, TRI_BITMASK=0x10, SAW_BITMASK=0x20,
	PULSE_BITMASK=0x40, NOISE_BITMASK=0x80, HOLDZERO_BITMASK=0x10,
	DECAYSUSTAIN_BITMASK=0x40, ATTACK_BITMASK=0x80, LOWPASS_BITMASK=0x10,
	BANDPASS_BITMASK=0x20, HIGHPASS_BITMASK=0x40, OFF3_BITMASK=0x80 };

static unsigned int TriSaw_8580[4096], PulseSaw_8580[4096], PulseTriSaw_8580[4096];
static const int ADSRperiods[16] = {9, 32, 63, 95, 149, 220, 267, 313, 392, 977, 1954, 3126, 3907, 11720, 19532, 31251};
static const byte ADSR_exptable[256] = {1, 30, 30, 30, 30, 30, 30, 16, 16, 16, 16, 16, 16, 16, 16, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 4, 4, 4, 4, 4, //pos0:1  pos6:30  pos14:16  pos26:8
	4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, //pos54:4 //pos93:2
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

static unsigned int combinedWF(SidChip* This, 
	unsigned int* wfarray, int index, char differ6581)
{
	if(differ6581 && This->model_6581) index&=0x7FF; 
	return wfarray[index];
}

static void createCombinedWF(unsigned int* wfarray, float bitmul, float bitstrength, float treshold)
{
	int  i,j,k;
	for (i=0; i<4096; i++) { wfarray[i]=0; for (j=0; j<12;j++) {
		float bitlevel=0; for (k=0; k<12; k++) {
			bitlevel += ( bitmul/pow(bitstrength,fabs(k-j)) ) * (((i>>k)&1)-0.5) ;
		}
		wfarray[i] += (bitlevel>=treshold)? pow(2,j) : 0; } wfarray[i]*=12;  }
}

void jsSID_calCuto(SidChip* This)
{
	float tmp = (This->regs[0] & 7) / 8.0 + This->regs[1] + 0.2;
	if (!This->model_6581) {
		This->cutoff = 1 - exp(tmp * This->cutoff_ratio);
	} else {
		This->cutoff = This->cutoff_bias + ( (tmp < 24)
			? 0 : 1 - exp((tmp-24) * This->cutoff_ratio ));
	}
}

void jsSID_calcReso(SidChip* This)
{
	int value = This->regs[2] >> 4;
	if(!This->model_6581) {
		This->resonance = pow(2, ((4 - value) / 8.0));
	} else {
		This->resonance = (value > 5) ? 8.0 / (value) : 1.41;
	}
}

void jsSID_init(SidChip* This,
	int samplerate, int model)
{
	memset(This, 0, sizeof(*This));
	for(int i = 0; i < 3; i++) {
		This->chnl[i].noise_LFSR = 0x7FFFF8;
 		This->chnl[i].muteMask = 1; }
	This->rand_state = 0;

	// filter state
	This->model_6581 = model;
	float cutoff_const = !This->model_6581 ? 
		(-2 * 3.14 * (12500.0 / 256)) : -2 * 3.14 * (20000.0 / 256);
	This->cutoff_ratio = cutoff_const / samplerate;
	This->cutoff_bias = 1 - exp( -2 * 3.14 * 220 / samplerate ); //around 220Hz below treshold
	jsSID_calCuto(This); jsSID_calcReso(This);
	
	// combined waveforms
	static char builtWaveforms=0;
	if(!builtWaveforms){
		createCombinedWF(TriSaw_8580, 0.8, 2.4, 0.64);
		createCombinedWF(PulseSaw_8580, 1.4, 1.9, 0.68);
		createCombinedWF(PulseTriSaw_8580, 0.8, 2.5, 0.64);
		builtWaveforms=1;   //This is used to build the waveform only once to speed up a future re-initialization
	}
}

void jsSID_write(SidChip* This,
	int addr, int data)
{
	SidChnl* chnl = This->chnl;
	if(addr < 0x15) 
	{	
		// update channel register
		while(addr >= 7) { addr -= 7; chnl++; }
		byte diff = chnl->regs[addr] ^ data;
		chnl->regs[addr] = data; 
		
		// update ADSR state
		if((addr == 4)&&(diff & GATE_BITMASK)){ 
			if (data & GATE_BITMASK) { chnl->ADSRstate = (ATTACK_BITMASK 
				| DECAYSUSTAIN_BITMASK); goto ADSR_ATTACK; }
			else { chnl->ADSRstate &= ~(ATTACK_BITMASK
				| DECAYSUSTAIN_BITMASK); goto ADSR_RELEASE; }
		}
				
		// update ADSR period	
		else if(addr > 4) {
			if (chnl->ADSRstate & ATTACK_BITMASK) { ADSR_ATTACK: 
				chnl->period = ADSRperiods[ chnl->regs[5] >> 4 ]; }
			else if (chnl->ADSRstate & DECAYSUSTAIN_BITMASK) { 
				chnl->period = ADSRperiods[ chnl->regs[5] & 0xF ]; }
			else { ADSR_RELEASE: chnl->period =	ADSRperiods[ chnl->regs[6] & 0xF ]; }
		}
		
	} else {
		// filters and resonance
		byte diff = This->regs[addr-0x15] ^ data;
		if(diff) { This->regs[addr-0x15] = data;
		if(addr < 0x17) { jsSID_calCuto(This); nothing(); 
		} else if(addr == 0x17) { 
			if(diff>>4) jsSID_calcReso(This);
			data = ~data & 7; for(int i = 0; i < 3; i++) { 
				chnl[i].muteMask ^= data; data >>= 1; }
		} else { chnl[2].muteMask ^= diff & 0x80; }}
	}
}
 
void jsSID_setMuteMask(SidChip* This, int mute_mask) {
	for(int i = 0; i < 3; i++) { This->chnl[i].muteMask &= 0xEF;
		if(mute_mask & (1<<i)) This->chnl[i].muteMask |= 0x10; }
}

static inline
u32 jsSID_triangle(SidChip* This, byte ctrl, u32 phaseaccu)
{
	int tmp = phaseaccu ^ (ctrl & RING_BITMASK ? This->sourceMSB : 0);
	return (((tmp < 0) ? ~phaseaccu : phaseaccu) >> 15) & 0xFFFF;
}

static inline
u32 jsSID_pulse(SidChnl* chnl, 
	byte test, u32 phaseaccu)
{
	u32 pw = READ_LIL16(&chnl->regs[2]) & 0xFFF;
	u32 tmp = phaseaccu >> 20;
	return ((tmp>=pw) || test) ? 0xFFFF : 0x0000;
}

float jsSID_render(SidChip* This)
{
	int channel;
	unsigned int period, accuadd, pw, wfout;
	u32 MSB;
	float filtin, output, ftmp; 

	filtin=output=0; 
	for (channel = 0; channel < SID_CHANNEL_AMOUNT; channel++) {
		SidChnl* chnl = &This->chnl[channel];
		
		// gcc register alloctor is braindead
		// we need to hold its hand
		#if __GNUC__ && __i386__
			asm("" : : "S"(chnl), "D"(This));
		#endif
	
		//ADSR envelope generator:
		#define SR chnl->regs[6]
		chnl->ratecnt++; chnl->ratecnt&=0x7FFF;   //can wrap around (ADSR delay-bug: short 1st frame)
		if (chnl->ratecnt == chnl->period) { //ratecounter shot (matches rateperiod) (in genuine SID ratecounter is LFSR)
			chnl->ratecnt = 0; //reset rate-counter on period-match
			if ((chnl->ADSRstate & ATTACK_BITMASK) || ++chnl->expcnt == ADSR_exptable[chnl->envcnt]) {
				chnl->expcnt = 0; 
				if (!(chnl->ADSRstate & HOLDZERO_BITMASK)) {
					if (chnl->ADSRstate & ATTACK_BITMASK) {
						if (chnl->envcnt!=0xFF) chnl->envcnt++; 
						else { chnl->ADSRstate &= 0xFF - ATTACK_BITMASK;
							chnl->period = ADSRperiods[ chnl->regs[5] & 0xF ]; }
					} else if ( !(chnl->ADSRstate & DECAYSUSTAIN_BITMASK) || chnl->envcnt != (SR >> 4) + (SR & 0xF0) ) {
						if (chnl->envcnt!=0) chnl->envcnt--;
						else chnl->ADSRstate |= HOLDZERO_BITMASK;
					}
				}
			}
		}
		
		//WAVE generation codes (phase accumulator and waveform-selector):
		#define ctrl chnl->regs[4]
		#define test (chnl->regs[4] & TEST_BITMASK)
		#define wf (chnl->regs[4] & 0xF0)
		
		u32 prevaccu = chnl->phaseaccu; u32 phaseaccu;
		accuadd = READ_LIL16(&chnl->regs[0]) << 8;
		if (test || (This->sourceMSBrise && (ctrl & SYNC_BITMASK))) {
			phaseaccu = 0;
		} else {
			phaseaccu = prevaccu + accuadd; 
		}
		chnl->phaseaccu = phaseaccu;
		This->sourceMSBrise = (~prevaccu & phaseaccu) >> 31;

		if (wf & NOISE_BITMASK) {
			int tmp = chnl->noise_LFSR;
			if (((phaseaccu & 0x10000000) != (prevaccu & 0x10000000))) { 
				int step = ((tmp >> 22)	^ (tmp >> 17)) & 0x1;
				tmp = (tmp << 1) | step; chnl->noise_LFSR = tmp; 
			}
			wfout = (wf & 0x70) ? 0 : ((tmp & 0x100000) >> 5) + ((tmp & 0x40000) >> 4) + ((tmp & 0x4000) >> 1) + ((tmp & 0x800) << 1) + ((tmp & 0x200) << 2) + ((tmp & 0x20) << 5) + ((tmp & 0x04) << 7) + ((tmp & 0x01) << 8);
		} else if (wf & PULSE_BITMASK) {
			wfout = jsSID_pulse(chnl, test, phaseaccu);
			if(wfout && (wf & ~PULSE_BITMASK)) { //combined pulse
				if (wf & TRI_BITMASK) { if (wf & SAW_BITMASK) {
					wfout = combinedWF(This, PulseTriSaw_8580, phaseaccu >> 20, 1);
				} else { wfout = combinedWF(This, PulseSaw_8580,
					jsSID_triangle(This, ctrl, phaseaccu) >> 4, 0);
				}} else { wfout = combinedWF(This, PulseSaw_8580, phaseaccu >> 20, 1);
				}
			}
		} //pulse+saw
		else if (wf & SAW_BITMASK) { 
			wfout = phaseaccu >> 16; //saw
			if (wf & TRI_BITMASK) wfout = combinedWF(This, TriSaw_8580, wfout >> 4, 1); //saw+triangle
		}
		else if (wf & TRI_BITMASK) {
			wfout = jsSID_triangle(This, ctrl, phaseaccu);
		} else {
			wfout = chnl->prevwfout;		
		} chnl->prevwfout = wfout; //emulate waveform 00 floating wave-DAC
		
		This->sourceMSB = phaseaccu; 
		ftmp = ((s32)wfout - 0x8000) * chnl->envcnt;
		if(!(chnl->muteMask & 0x11)) filtin += ftmp;
		else if(!(chnl->muteMask & 0x90)) output += ftmp;
	}
	
	//FILTER:
	filtin += *(float*)&This->rand_state; This->rand_state ^= 0x37800000;
	ftmp = filtin + This->prevbandpass * This->resonance + This->prevlowpass;
	if (This->regs[3] & HIGHPASS_BITMASK) output -= ftmp;
	ftmp = This->prevbandpass - ftmp * This->cutoff;
	This->prevbandpass = ftmp;
	if (This->regs[3] & BANDPASS_BITMASK) output -= ftmp;
	ftmp = This->prevlowpass + ftmp * This->cutoff;
	This->prevlowpass = ftmp;
	if (This->regs[3] & LOWPASS_BITMASK) output += ftmp; 
	return output * (This->regs[3] & 0xF);
}
