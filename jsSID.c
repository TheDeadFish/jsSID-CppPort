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

#include <math.h>
#include <string.h>
#include <stdio.h>

typedef unsigned char byte;

enum { C64_PAL_CPUCLK = 985248,	SID_CHANNEL_AMOUNT = 3,
	OUTPUT_SCALEDOWN = 0x10000 * SID_CHANNEL_AMOUNT * 16};

enum { GATE_BITMASK=0x01, SYNC_BITMASK=0x02, RING_BITMASK=0x04,
		TEST_BITMASK=0x08, TRI_BITMASK=0x10, SAW_BITMASK=0x20,
		PULSE_BITMASK=0x40, NOISE_BITMASK=0x80, HOLDZERO_BITMASK=0x10,
		DECAYSUSTAIN_BITMASK=0x40, ATTACK_BITMASK=0x80, LOWPASS_BITMASK=0x10,
		BANDPASS_BITMASK=0x20, HIGHPASS_BITMASK=0x40, OFF3_BITMASK=0x80 };

static const byte FILTSW[9] = {1,2,4,1,2,4,1,2,4};

static char buildWaveforms=1;
static byte sidRegs[3][25];
static byte ADSRstate[9]; static double ratecnt[9]; static int envcnt[9];
static byte expcnt[9]; static byte prevSR[9]; static double phaseaccu[9];
static int prevaccu[9]; static byte sourceMSBrise[9]; static int sourceMSB[3];
static int noise_LFSR[9]; static double prevwfout[9]; static int prevwavdata[9];
static double prevlowpass[3]; static double prevbandpass[3];
static double cutoff_ratio_8580; static double cutoff_ratio_6581;

static int SID_model; static double clk_ratio;
static int TriSaw_8580[4096];
static int PulseSaw_8580[4096];
static int PulseTriSaw_8580[4096];
static int muteMask_c64=0;
static double ADSRperiods[16] = {9, 32, 63, 95, 149, 220, 267, 313, 392, 977, 1954, 3126, 3907, 11720, 19532, 31251};
static byte ADSRstep[16] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
static const byte ADSR_exptable[256] = {1, 30, 30, 30, 30, 30, 30, 16, 16, 16, 16, 16, 16, 16, 16, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 4, 4, 4, 4, 4, //pos0:1  pos6:30  pos14:16  pos26:8
	4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, //pos54:4 //pos93:2
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

static int combinedWF(int channel, int* wfarray, int index, char differ6581)
{
	if(differ6581 && SID_model==6581) index&=0x7FF; int combiwf = (wfarray[index]
		+prevwavdata[channel])/2; prevwavdata[channel]=wfarray[index]; return combiwf;
}

static void createCombinedWF(int* wfarray, double bitmul, double bitstrength, double treshold)
{
	for (int i=0; i<4096; i++) { wfarray[i]=0; for (int j=0; j<12;j++) {
		double bitlevel=0; for (int k=0; k<12; k++) {
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
	SID_model = model==1?6581:8580;
	clk_ratio = (double)C64_PAL_CPUCLK / samplerate;
	double period0 = fmax(clk_ratio, 9);
	ADSRperiods[0] = period0; ADSRstep[0] = ceil(period0/9);
	cutoff_ratio_8580 = -2 * 3.14 * (12500.0 / 256) / samplerate;
	cutoff_ratio_6581 = -2 * 3.14 * (20000.0 / 256) / samplerate;

	if(buildWaveforms){
		createCombinedWF(TriSaw_8580, 0.8, 2.4, 0.64);
		createCombinedWF(PulseSaw_8580, 1.4, 1.9, 0.68);
		createCombinedWF(PulseTriSaw_8580, 0.8, 2.5, 0.64);
		buildWaveforms=0;   //This is used to build the waveform only once to speed up a future re-initialization
	}

	for(int i = 0; i < 9; i++) {
		ADSRstate[i] = HOLDZERO_BITMASK; ratecnt[i] = 0; envcnt[i] = 0;
		expcnt[i] = 0; prevSR[i] = 0; phaseaccu[i] = 0; prevaccu[i] = 0;
		noise_LFSR[i] = 0x7FFFF8; prevwfout[i] = 0; prevwavdata[i] = 0;
	}
	for(int i = 0; i < 3; i++) {
		sourceMSBrise[i] = 0; sourceMSB[i] = 0;
		prevlowpass[i] = 0; prevbandpass[i] = 0;
	}
}

void jsSID_write(int chip, int addr, int data)
{
	sidRegs[chip][addr] = data;
}

double jsSID_render(int num)
{
	double filtin = 0, output = 0;
	int step;
	byte * sReg = sidRegs[num];
	byte * vReg = sReg;
	for (int channel = num * SID_CHANNEL_AMOUNT; channel <(num + 1) * SID_CHANNEL_AMOUNT; channel++, vReg += 7) {
		byte ctrl = vReg[4];
		int tmp = 0;
		//ADSR envelope generator:
		{
			double period;
			byte SR = vReg[6];
			byte prevgate = (ADSRstate[channel] & GATE_BITMASK);
			if (prevgate != (ctrl & GATE_BITMASK)) { //gatebit-change?
				if (prevgate) {
					ADSRstate[channel] &= 0xFF - (GATE_BITMASK | ATTACK_BITMASK | DECAYSUSTAIN_BITMASK);
				} //falling edge
				else {
					ADSRstate[channel] = (GATE_BITMASK | ATTACK_BITMASK | DECAYSUSTAIN_BITMASK); //rising edge, also sets hold_zero_bit=0
					if ((SR & 0xF) > (prevSR[channel] & 0xF)) tmp = 1;
				}
			}
			prevSR[channel] = SR; //if(SR&0xF) ratecnt[channel]+=5;  //assume SR->GATE write order: workaround to have crisp soundstarts by triggering delay-bug
			ratecnt[channel] += clk_ratio;
			if (ratecnt[channel] >= 0x8000) ratecnt[channel] -= 0x8000; //can wrap around (ADSR delay-bug: short 1st frame)
			if (ADSRstate[channel] & ATTACK_BITMASK) {
				step = vReg[5] >> 4;
				period = ADSRperiods[step];
			} else if (ADSRstate[channel] & DECAYSUSTAIN_BITMASK) {
				step = vReg[5] & 0xF;
				period = ADSRperiods[step];
			} else {
				step = SR & 0xF;
				period = ADSRperiods[step];
			}
			step = ADSRstep[step];
			if (ratecnt[channel] >= period && ratecnt[channel] < period + clk_ratio && tmp == 0) { //ratecounter shot (matches rateperiod) (in genuine SID ratecounter is LFSR)
				ratecnt[channel] -= period; //compensation for timing instead of simply setting 0 on rate-counter overload
				if ((ADSRstate[channel] & ATTACK_BITMASK) || ++expcnt[channel] == ADSR_exptable[envcnt[channel]]) {
					if (!(ADSRstate[channel] & HOLDZERO_BITMASK)) {
						if (ADSRstate[channel] & ATTACK_BITMASK) {
							envcnt[channel] += step;
							if (envcnt[channel] >= 0xFF) {
								envcnt[channel] = 0xFF;
								ADSRstate[channel] &= 0xFF - ATTACK_BITMASK;
							}
						} else if (!(ADSRstate[channel] & DECAYSUSTAIN_BITMASK) || envcnt[channel] > (SR >> 4) + (SR & 0xF0)) {
							envcnt[channel] -= step;
							if (envcnt[channel] <= 0 && envcnt[channel] + step != 0) {
								envcnt[channel] = 0;
								ADSRstate[channel] |= HOLDZERO_BITMASK;
							}
						}
					}
					expcnt[channel] = 0;
				}
			}
			envcnt[channel] &= 0xFF;
		}

		//WAVE generation codes (phase accumulator and waveform-selector):
		int pw;
		double lim, fstep, wfout;
		byte test = ctrl & TEST_BITMASK;
		byte wf = ctrl & 0xF0;
		double accuadd = (vReg[0] + vReg[1] * 256) * clk_ratio;
		if (test || ((ctrl & SYNC_BITMASK) && sourceMSBrise[num])) {
			phaseaccu[channel] = 0;
		} else {
			phaseaccu[channel] += accuadd;
			if (phaseaccu[channel] > 0xFFFFFF) phaseaccu[channel] -= 0x1000000;
		}
		int iPhaseaccu = phaseaccu[channel];
		int MSB = iPhaseaccu & 0x800000;
		sourceMSBrise[num] = (MSB > (prevaccu[channel] & 0x800000)) ? 1 : 0; //phaseaccu[channel] &= 0xFFFFFF;
		if (wf & NOISE_BITMASK) {
			tmp = noise_LFSR[channel];
			if (((iPhaseaccu & 0x100000) != (prevaccu[channel] & 0x100000)) || accuadd >= 0x100000) { //clock LFSR all time if clockrate exceeds observable at given samplerate
				step = (tmp & 0x400000) ^ ((tmp & 0x20000) << 5);
				tmp = ((tmp << 1) + (step ? 1 : test)) & 0x7FFFFF;
				noise_LFSR[channel] = tmp;
			}
			wfout = (wf & 0x70) ? 0 : ((tmp & 0x100000) >> 5) + ((tmp & 0x40000) >> 4) + ((tmp & 0x4000) >> 1) + ((tmp & 0x800) << 1) + ((tmp & 0x200) << 2) + ((tmp & 0x20) << 5) + ((tmp & 0x04) << 7) + ((tmp & 0x01) << 8);
		} else if (wf & PULSE_BITMASK) {
			pw = (vReg[2] + (vReg[3] & 0xF) * 256) * 16;
			tmp = (int) accuadd >> 9;
			if (0 < pw && pw < tmp) pw = tmp;
			tmp ^= 0xFFFF;
			if (pw > tmp) pw = tmp;
			tmp = iPhaseaccu >> 8;
			if (wf == PULSE_BITMASK) {
				fstep = 256.0 / ((int) accuadd >> 16); //simple pulse, most often used waveform, make it sound as clean as possible without oversampling
				if (test) wfout = 0xFFFF;
				else if (tmp < pw) {
					lim = (0xFFFF - pw) * fstep;
					if (lim > 0xFFFF) lim = 0xFFFF;
					wfout = lim - (pw - tmp) * fstep;
					if (wfout < 0) wfout = 0;
				} //rising edge
				else {
					lim = pw * fstep;
					if (lim > 0xFFFF) lim = 0xFFFF;
					wfout = (0xFFFF - tmp) * fstep - lim;
					if (wfout >= 0) wfout = 0xFFFF;
					wfout = (int) wfout & 0xFFFF;
				}
			} //falling edge
			else { //combined pulse
				wfout = (tmp >= pw || test) ? 0xFFFF : 0; //(this would be enough for simple but aliased-at-high-pitches pulse)
				if (wf & TRI_BITMASK) {
					if (wf & SAW_BITMASK) {
						wfout = (wfout) ? combinedWF(channel, PulseTriSaw_8580, tmp >> 4, 1) : 0;
					} //pulse+saw+triangle (waveform nearly identical to tri+saw)
					else {
						tmp = iPhaseaccu ^ (ctrl & RING_BITMASK ? sourceMSB[num] : 0);
						wfout = (wfout) ? combinedWF(channel, PulseSaw_8580, (tmp ^ (tmp & 0x800000 ? 0xFFFFFF : 0)) >> 11, 0) : 0;
					}
				} //pulse+triangle
				else if (wf & SAW_BITMASK) wfout = (wfout) ? combinedWF(channel, PulseSaw_8580, tmp >> 4, 1) : 0;
			}
		} //pulse+saw
		else if (wf & SAW_BITMASK) {
			wfout = iPhaseaccu >> 8; //saw (this row would be enough for simple but aliased-at-high-pitch saw)
			if (wf & TRI_BITMASK) wfout = combinedWF(channel, TriSaw_8580, (int) wfout >> 4, 1); //saw+triangle
			else {
				fstep = accuadd / 0x1200000;
				wfout += wfout * fstep;
				if (wfout > 0xFFFF) wfout = 0xFFFF - (wfout - 0x10000) / fstep;
			}
		} //simple cleaned (bandlimited) saw
		else if (wf & TRI_BITMASK) {
			tmp = iPhaseaccu ^ (ctrl & RING_BITMASK ? sourceMSB[num] : 0);
			wfout = (tmp ^ (tmp & 0x800000 ? 0xFFFFFF : 0)) >> 7;
		}
		if (wf) prevwfout[channel] = wfout;
		else {
			wfout = prevwfout[channel];
		} //emulate waveform 00 floating wave-DAC
		prevaccu[channel] = iPhaseaccu;
		sourceMSB[num] = MSB;
		if (!(muteMask_c64 & FILTSW[channel])) {
			if (sReg[0x17] & FILTSW[channel]) filtin += (wfout - 0x8000) * (envcnt[channel] / 256.0);
			else if ((FILTSW[channel] != 4) || !(sReg[0x18] & OFF3_BITMASK)) output += (wfout - 0x8000) * (envcnt[channel] / 256.0);
		}
	}

	//FILTER:
	double ftmp, resonance, cutoff = (sReg[0x15] & 7) / 8.0 + sReg[0x16] + 0.2;
	if (SID_model == 8580) {
		cutoff = 1 - exp(cutoff * cutoff_ratio_8580);
		resonance = pow(2, ((4 - (sReg[0x17] >> 4)) / 8.0));
	} else {
		if (cutoff < 24) cutoff = 0.035;
		else cutoff = 1 - 1.263 * exp(cutoff * cutoff_ratio_6581);
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
	return (output / OUTPUT_SCALEDOWN) * (sReg[0x18] & 0xF); // master output
}
