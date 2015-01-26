#include "pitch-detect.h"
#include "uafunc.h"
#include "pitches.h"


#ifdef PITCH_DETECT_FFT
#undef PITCH_DETECT_AUTO
#include "kiss_fftr.h"

volatile kiss_fft_scalar data[N_POINTS];
volatile kiss_fft_cpx fftout[N_POINTS/2 + 1];
volatile uint16_t i = 0;

//Smooth the results somewhat
#define NUM_SMOOTH 3
float smoothFreq[NUM_SMOOTH];
short currentSmooth = 0;

//mem needs to be:
//subsize = sizeof(struct kiss_fft_state) + sizeof(kiss_fft_cpx)*(nfft-1); /* twiddle factors*/
//memneeded = sizeof(struct kiss_fftr_state) + subsize + sizeof(kiss_fft_cpx) * ( nfft * 3 / 2);
size_t memSize = sizeof(struct kiss_fftr_state) + sizeof(struct kiss_fft_state) + sizeof(kiss_fft_cpx)*(N_POINTS-1) + sizeof(kiss_fft_cpx) * (N_POINTS * 3 / 2);
uint8_t mem[sizeof(struct kiss_fftr_state) + sizeof(struct kiss_fft_state) + sizeof(kiss_fft_cpx)*(N_POINTS-1) + sizeof(kiss_fft_cpx) * (N_POINTS * 3 / 2)];
kiss_fftr_cfg cfg;

int pitchDetectInit()
{
	//memSize = sizeof(struct kiss_fftr_state) + sizeof(struct kiss_fft_state) + sizeof(kiss_fft_cpx)*(N_POINTS-1) + sizeof(kiss_fft_cpx) * (N_POINTS * 3 / 2);
	//mem[sizeof(struct kiss_fftr_state) + sizeof(struct kiss_fft_state) + sizeof(kiss_fft_cpx)*(N_POINTS-1) + sizeof(kiss_fft_cpx) * (N_POINTS * 3 / 2)];
	cfg = kiss_fftr_alloc(N_POINTS, 0, (void*)mem, &memSize);
	
	return (cfg == NULL);
}

float pitchDetect()
{	
	//Wait for data collection to finish
	//Use WFI instruction to sleep as much as possible
	while(i < N_POINTS)
	{
		__WFI();
	}
	
	//To frequency domain
	kiss_fftr(cfg, data, fftout);
	
	
	//Find the peak, find main frequency
	//Ignore DC
	int peakidx = 3;
	int peakval = (int)sqrt(fftout[peakidx].r * fftout[peakidx].r + fftout[peakidx].i * fftout[peakidx].i);	
	{
		int r;
		//Don't go all the way to the last point because it's an edge case.
		//We can't average with its neighbor, and it's at the fringe of the Nyquist rate anyway
		for(r = peakidx + 1; r < N_POINTS/2/* + 1*/; r++)
		{
			int vali = (int)sqrt(fftout[r].r * fftout[r].r + fftout[r].i * fftout[r].i);
			if(vali > peakval)
			{
				peakidx = r;
				peakval = vali;
			}
		}
	}
	
	if(peakval < NOISE_FLOOR)
	{
		return RET_NO_PITCH;
	}

	//Using the weighted average of 5 values seems to produce a better result than for 3
	//Maximum error reduced, both in pure-tone and in a slightly noisy scenario
	float left = sqrt(fftout[peakidx-1].r * fftout[peakidx-1].r + fftout[peakidx-1].i * fftout[peakidx-1].i);
	float right = sqrt(fftout[peakidx+1].r * fftout[peakidx+1].r + fftout[peakidx+1].i * fftout[peakidx+1].i);
	float mainFreqf = (left * (peakidx-1) + right * (peakidx+1) + peakidx * peakval) * (float)SAMPLE_FREQ/(float)N_POINTS / (left + peakval + right);

	//These lines seem to help precision by up to a cent in the worst cases
	//float left2 = sqrt(fftout[peakidx-2].r * fftout[peakidx-2].r + fftout[peakidx-2].i * fftout[peakidx-2].i);
	//float right2 = sqrt(fftout[peakidx+2].r * fftout[peakidx+2].r + fftout[peakidx+2].i * fftout[peakidx+2].i);
	//float mainFreqf = (left2 * (peakidx-2) + left * (peakidx-1) + right * (peakidx+1) + right2 * (peakidx+2) + peakidx * peakval) * SAMPLE_FREQ/N_POINTS / (left + peakval + right + left2 + right2);

	
	smoothFreq[currentSmooth] = mainFreqf;
	currentSmooth = (currentSmooth + 1) % NUM_SMOOTH;
	
	
	//Calculate a running average of samples; try to further enhance precision
	{
		mainFreqf = 0.0f;
		short s;
		for(s = 0; s < NUM_SMOOTH; s++)
		{
			mainFreqf += smoothFreq[s];
		}
		mainFreqf /= ((float)NUM_SMOOTH);
	}
	
	return mainFreqf;
}
#endif


#ifdef PITCH_DETECT_AUTO
float hanning[N_POINTS];
int16_t data[N_POINTS];

int pitchDetectInit()
{
	//Fill in hanning window
	int k;
	for(k = 0; k < N_POINTS; k++)
	{
		hanning[k] = 0.5f * (1.0f - cosf(2.0f * M_PI * k / (N_POINTS - 1)));
	}
	
	/*
	This code will calculate hanning autocorrelation, if that turns out to be necessary
	int j;
	for(k = 0; k < N_POINTS; k++)
	{
		for(j = 0; j + k < N_POINTS; j++)
		{
			hanningAuto[k] += hanning[j] * hanning[j+k];
		}
	}
	*/
	
	return 0;
}

float pitchDetect()
{
	//Wait for data collection to finish
	while(i < N_POINTS) {}
	
	//Filter with hanning window
	for(i = 0; i < N_POINTS; i++)
	{
		data[i] *= hanning[i];
	}
}
#endif
