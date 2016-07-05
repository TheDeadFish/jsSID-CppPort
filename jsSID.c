// jsSID port by Hermit (Mihaly Horvath) to C, (Year 2016) http://hermit.sidrip.com
// Converted to C/C++ by DeadFish Shitware & Delek, cycle-exact version corrected/finalized by Hermit

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

typedef unsigned char byte;
typedef unsigned int u32;
typedef signed int s32;

enum { C64_PAL_CPUCLK = 985248,	SID_CHANNEL_AMOUNT = 3, 
	OUTPUT_SCALEDOWN = 0x10000 * SID_CHANNEL_AMOUNT * 16};

enum { GATE_BITMASK=0x01, SYNC_BITMASK=0x02, RING_BITMASK=0x04,
	TEST_BITMASK=0x08, TRI_BITMASK=0x10, SAW_BITMASK=0x20,
	PULSE_BITMASK=0x40, NOISE_BITMASK=0x80, HOLDZERO_BITMASK=0x10,
	DECAYSUSTAIN_BITMASK=0x40, ATTACK_BITMASK=0x80, LOWPASS_BITMASK=0x10,
	BANDPASS_BITMASK=0x20, HIGHPASS_BITMASK=0x40, OFF3_BITMASK=0x80 };


//'static' variables and fuctions will only be seen by the current file (or compilation unit) unlike simple global variables or 'extern' ones
static const byte FILTSW[9] = {1,2,4,1,2,4,1,2,4};

static char buildWaveforms=1;
static byte sidRegs[3][25], ADSRstate[9], envcnt[9], expcnt[9], sourceMSBrise[9];  
static unsigned int ratecnt[9], prevwfout[9]; 
static u32 phaseaccu[9], prevaccu[9], sourceMSB[3], noise_LFSR[9];
static float prevlowpass[3], prevbandpass[3], cutoff_ratio_8580, cutoff_ratio_6581, cutoff_bias_6581;
static int SID_model, muteMask_c64=0;
static unsigned int TriSaw_8580[4096], PulseSaw_8580[4096], PulseTriSaw_8580[4096];
static int ADSRperiods[16] = {9, 32, 63, 95, 149, 220, 267, 313, 392, 977, 1954, 3126, 3907, 11720, 19532, 31251};
static const byte ADSR_exptable[256] = {1, 30, 30, 30, 30, 30, 30, 16, 16, 16, 16, 16, 16, 16, 16, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 4, 4, 4, 4, 4, //pos0:1  pos6:30  pos14:16  pos26:8
	4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, //pos54:4 //pos93:2
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };


static unsigned int combinedWF(int channel, unsigned int* wfarray, int index, char differ6581)
{
	if(differ6581 && SID_model==6581) index&=0x7FF; 
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


float jsSID_getVolume(int channel)
{
	if(sidRegs[0][0x18]&0xF==0)return 0.0f;
	return envcnt[channel]/256.0f;
}

void jsSID_setMuteMask(int mute_mask)
{
	muteMask_c64=mute_mask;
}

void jsSID_init(int samplerate, int model)
{
	int i;
	SID_model = model==1?6581:8580;
	cutoff_ratio_8580 = -2 * 3.14 * (12500.0 / 256) / samplerate;
	cutoff_ratio_6581 = -2 * 3.14 * (20000.0 / 256) / samplerate;
	cutoff_bias_6581 = 1 - exp( -2 * 3.14 * 220 / samplerate ); //around 220Hz below treshold
	
	if(buildWaveforms){
		createCombinedWF(TriSaw_8580, 0.8, 2.4, 0.64);
		createCombinedWF(PulseSaw_8580, 1.4, 1.9, 0.68);
		createCombinedWF(PulseTriSaw_8580, 0.8, 2.5, 0.64);
		buildWaveforms=0;   //This is used to build the waveform only once to speed up a future re-initialization
	}
	
	for(i = 0; i < 9; i++) {
		ADSRstate[i] = HOLDZERO_BITMASK; envcnt[i] = 0; ratecnt[i] = 0; 
		phaseaccu[i] = 0; prevaccu[i] = 0; expcnt[i] = 0; 
		noise_LFSR[i] = 0x7FFFF8; prevwfout[i] = 0;
	}
	for(i = 0; i < 3; i++) {
		sourceMSBrise[i] = 0; sourceMSB[i] = 0;
		prevlowpass[i] = 0; prevbandpass[i] = 0;
	}
	for(i=0; i < 25; i++) { sidRegs[0][i]=0; sidRegs[1][i]=0; sidRegs[2][i]=0; }

}

void jsSID_write(int chip, int addr, int data)
{
	sidRegs[chip][addr] = data;
}


float jsSID_render(int num)
{
	byte channel, ctrl, SR, prevgate, wf, test; 
	byte *sReg, *vReg;
	unsigned int period, accuadd, pw, wfout;
	u32 MSB;
	float filtin, output, ftmp, resonance, cutoff; 

	filtin=output=0; sReg = sidRegs[num]; vReg = sReg;
	for (channel = num * SID_CHANNEL_AMOUNT; channel < (num + 1) * SID_CHANNEL_AMOUNT; channel++, vReg += 7) {
		ctrl = vReg[4];

		//ADSR envelope generator:
		{
			SR = vReg[6];
			prevgate = (ADSRstate[channel] & GATE_BITMASK);
			if (prevgate != (ctrl & GATE_BITMASK)) { //gatebit-change?
				if (prevgate) {
					ADSRstate[channel] &= 0xFF - (GATE_BITMASK | ATTACK_BITMASK | DECAYSUSTAIN_BITMASK);
				} //falling edge
				else {
					ADSRstate[channel] = (GATE_BITMASK | ATTACK_BITMASK | DECAYSUSTAIN_BITMASK); //rising edge, also sets hold_zero_bit=0
				}
			}
			if (ADSRstate[channel] & ATTACK_BITMASK) period = ADSRperiods[ vReg[5] >> 4 ];
		else if (ADSRstate[channel] & DECAYSUSTAIN_BITMASK) period = ADSRperiods[ vReg[5] & 0xF ];
		else period = ADSRperiods[ SR & 0xF ];
			ratecnt[channel]++; ratecnt[channel]&=0x7FFF;   //can wrap around (ADSR delay-bug: short 1st frame)
			if (ratecnt[channel] == period) { //ratecounter shot (matches rateperiod) (in genuine SID ratecounter is LFSR)
				ratecnt[channel] = 0; //reset rate-counter on period-match
				if ((ADSRstate[channel] & ATTACK_BITMASK) || ++expcnt[channel] == ADSR_exptable[envcnt[channel]]) {
					expcnt[channel] = 0; 
					if (!(ADSRstate[channel] & HOLDZERO_BITMASK)) {
						if (ADSRstate[channel] & ATTACK_BITMASK) {
							if (envcnt[channel]!=0xFF) envcnt[channel]++; 
							else ADSRstate[channel] &= 0xFF - ATTACK_BITMASK;
						} else if ( !(ADSRstate[channel] & DECAYSUSTAIN_BITMASK) || envcnt[channel] != (SR >> 4) + (SR & 0xF0) ) {
							if (envcnt[channel]!=0) envcnt[channel]--;
							else ADSRstate[channel] |= HOLDZERO_BITMASK;
						}
					}
				}
			}
			envcnt[channel] &= 0xFF;
		}
		
		//WAVE generation codes (phase accumulator and waveform-selector):
		test = ctrl & TEST_BITMASK;
		wf = ctrl & 0xF0;
		accuadd = (vReg[0] + vReg[1] * 256);
		if (test || ((ctrl & SYNC_BITMASK) && sourceMSBrise[num])) {
			phaseaccu[channel] = 0;
		} else {
			phaseaccu[channel] += accuadd; phaseaccu[channel]&=0xFFFFFF;
		}
		MSB = phaseaccu[channel] & 0x800000;
		sourceMSBrise[num] = (MSB > (prevaccu[channel] & 0x800000)) ? 1 : 0;
		if (wf & NOISE_BITMASK) {
			int tmp = noise_LFSR[channel];
			if (((phaseaccu[channel] & 0x100000) != (prevaccu[channel] & 0x100000))) { 
				int step = (tmp & 0x400000) ^ ((tmp & 0x20000) << 5);
				tmp = ((tmp << 1) + (step ? 1 : test)) & 0x7FFFFF;
				noise_LFSR[channel] = tmp;
			}
			wfout = (wf & 0x70) ? 0 : ((tmp & 0x100000) >> 5) + ((tmp & 0x40000) >> 4) + ((tmp & 0x4000) >> 1) + ((tmp & 0x800) << 1) + ((tmp & 0x200) << 2) + ((tmp & 0x20) << 5) + ((tmp & 0x04) << 7) + ((tmp & 0x01) << 8);
		} else if (wf & PULSE_BITMASK) {
			pw = (vReg[2] + (vReg[3] & 0xF) * 256) * 16;
			
			int tmp = phaseaccu[channel] >> 8;
			if (wf == PULSE_BITMASK) {
				if (test || tmp>=pw) wfout = 0xFFFF;
				else {
					wfout=0;
				}
			}
			else { //combined pulse
				wfout = (tmp >= pw || test) ? 0xFFFF : 0; //(this would be enough for simple but aliased-at-high-pitches pulse)
				if (wf & TRI_BITMASK) {
					if (wf & SAW_BITMASK) {
						wfout = (wfout) ? combinedWF(channel, PulseTriSaw_8580, tmp >> 4, 1) : 0;
					} //pulse+saw+triangle (waveform nearly identical to tri+saw)
					else {
						tmp = phaseaccu[channel] ^ (ctrl & RING_BITMASK ? sourceMSB[num] : 0);
						wfout = (wfout) ? combinedWF(channel, PulseSaw_8580, (tmp ^ (tmp & 0x800000 ? 0xFFFFFF : 0)) >> 11, 0) : 0;
					}
				} //pulse+triangle
				else if (wf & SAW_BITMASK) wfout = (wfout) ? combinedWF(channel, PulseSaw_8580, tmp >> 4, 1) : 0;
			}
		} //pulse+saw
		else if (wf & SAW_BITMASK) { 
			wfout = phaseaccu[channel] >> 8; //saw
			if (wf & TRI_BITMASK) wfout = combinedWF(channel, TriSaw_8580, wfout >> 4, 1); //saw+triangle
		}
		else if (wf & TRI_BITMASK) {
			int tmp = phaseaccu[channel] ^ (ctrl & RING_BITMASK ? sourceMSB[num] : 0);
			wfout = (tmp ^ (tmp & 0x800000 ? 0xFFFFFF : 0)) >> 7;
		}
		if (wf) prevwfout[channel] = wfout;
		else {
			wfout = prevwfout[channel];
		} //emulate waveform 00 floating wave-DAC
		prevaccu[channel] = phaseaccu[channel];
		sourceMSB[num] = MSB; 
		if (!(muteMask_c64 & FILTSW[channel])) {
			if (sReg[0x17] & FILTSW[channel]) filtin += ((s32)wfout - 0x8000) * (envcnt[channel] / 256.0);
			else if ((FILTSW[channel] != 4) || !(sReg[0x18] & OFF3_BITMASK)) 
				output += ((s32)wfout - 0x8000) * (envcnt[channel]) / 256.0;
		}
	}
	
	//FILTER:
	cutoff = (sReg[0x15] & 7) / 8.0 + sReg[0x16] + 0.2;
	if (SID_model == 8580) {
		cutoff = 1 - exp(cutoff * cutoff_ratio_8580);
		resonance = pow(2, ((4 - (sReg[0x17] >> 4)) / 8.0));
	} else {
		cutoff = cutoff_bias_6581 + ( (cutoff < 24) ? 0 : 1 - exp((cutoff-24) * cutoff_ratio_6581) );
		resonance = (sReg[0x17] > 0x5F) ? 8.0 / (sReg[0x17] >> 4) : 1.41;
	}
	ftmp = filtin + prevbandpass[num] * resonance + prevlowpass[num];
	if (sReg[0x18] & HIGHPASS_BITMASK) output -= ftmp;
	ftmp = prevbandpass[num] - ftmp * cutoff;
	prevbandpass[num] = ftmp;
	if (sReg[0x18] & BANDPASS_BITMASK) output -= ftmp;
	ftmp = prevlowpass[num] + ftmp * cutoff;
	prevlowpass[num] = ftmp;
	if (sReg[0x18] & LOWPASS_BITMASK) output += ftmp; 
	output = (output / OUTPUT_SCALEDOWN) * (sReg[0x18] & 0xF);
	if (output>=1.0) output=1.0; else if (output<=-1.0) output=-1.0; //saturation logic on overload (not needed if the callback handles it)
	return output; // master output
}
