#include "uafunc.h"
#include "kiss_fftr.h"
#include "_kiss_fft_guts.h"
#include "pitches.h"

#define LED_VF 0x00000040
#define LED_F 0x00000080
#define LED_IT 0x00000100
#define LED_S  0x00000200
#define LED_VS 0x00000400
#define LED_ALL 0x000007C0

#define CENT_VFVS 30
#define CENT_FS 20
#define CENT_IT 10

//NOTE: 256 points seems to be the max with 8k of RAM- ~800B short of 512...
//Works with frequencies up to 900 Hz (which is ~A5)
//FIXME: Trying lower sampling frequency- up to ~Bb4
//#define SAMPLE_FREQ 1800
#define SAMPLE_FREQ 1000
#define N_POINTS 256
volatile kiss_fft_scalar data[N_POINTS];
volatile kiss_fft_cpx fftout[N_POINTS/2 + 1];
volatile uint16_t i = 0;

void setup()
{
	LPC_IOCON->SWCLK_PIO0_10 |= 0x00000001;
	LPC_GPIO0->DATA = 0xFFFFFFFF;
	LPC_GPIO0->DIR = (1<<6)|(1<<7)|(1<<8)|(1<<9)|(1<<10); 
    
	UARTInit(115200);
	ADCInit(CHN0);
	Timer3Init(72000000/(2*SAMPLE_FREQ) - 1);
	Timer3Match0(1,INTERRUPT|RESET);//This will generate an interrupt every OTHER time the divisor is hit
}

void Timer3Interrupt0()
{
	data[i] = ADCRead(CHN0);
	i++;
	if(i >= N_POINTS) //we really just care about ==, but >= is less error-prone
	{
		Timer3Pause();
	}
}
  
void loop()
{
	//mem needs to be:
	//subsize = sizeof(struct kiss_fft_state) + sizeof(kiss_fft_cpx)*(nfft-1); /* twiddle factors*/
  //memneeded = sizeof(struct kiss_fftr_state) + subsize + sizeof(kiss_fft_cpx) * ( nfft * 3 / 2);
  
  size_t memSize = sizeof(struct kiss_fftr_state) + sizeof(struct kiss_fft_state) + sizeof(kiss_fft_cpx)*(N_POINTS-1) + sizeof(kiss_fft_cpx) * (N_POINTS * 3 / 2);
	uint8_t mem[sizeof(struct kiss_fftr_state) + sizeof(struct kiss_fft_state) + sizeof(kiss_fft_cpx)*(N_POINTS-1) + sizeof(kiss_fft_cpx) * (N_POINTS * 3 / 2)];
  
  
	kiss_fftr_cfg cfg = kiss_fftr_alloc(N_POINTS, 0, (void*)mem, &memSize);
	
	//Check for failure
	if(cfg == NULL)
	{
		LPC_GPIO0->DATA = 0xFFFFFFFF ^ ( LED_F | LED_S );
		while(1)
		{
		}
	}
	
	//Loop forever
	while(1)
	{	
		//Wait for data collection to finish
		while(i < N_POINTS)
		{
		}
		
		//To frequency domain
		kiss_fftr(cfg, data, fftout);
		
		
		//Find the peak, find main frequency
		//Ignore DC
		int peakidx = 3;
		int peakval = (int)sqrt(fftout[peakidx].r * fftout[peakidx].r + fftout[peakidx].i * fftout[peakidx].i);	
		{
			int r;
			for(r = peakidx + 1; r < N_POINTS/2 + 1; r++)
			{
				int vali = (int)sqrt(fftout[r].r * fftout[r].r + fftout[r].i * fftout[r].i);
				if(vali > peakval)
				{
					peakidx = r;
					peakval = vali;
				}
			}
		}
		
		float left = sqrt(fftout[peakidx-1].r * fftout[peakidx-1].r + fftout[peakidx-1].i * fftout[peakidx-1].i);
		float right = sqrt(fftout[peakidx+1].r * fftout[peakidx+1].r + fftout[peakidx+1].i * fftout[peakidx+1].i);
		float mainFreqf = (left * (peakidx-1) + right * (peakidx+1) + peakidx * peakval) * SAMPLE_FREQ/N_POINTS / (left + peakval + right);
		int mainFreq = (int)(mainFreqf + 0.5f);
		
		
		//FIXME: Debug
		/*{
			int t = 0;
			for(t = 0; t < N_POINTS/2 + 1; t++)
			{
				int vali = (int)sqrt(fftout[t].r * fftout[t].r + fftout[t].i * fftout[t].i);
				//int vali = data[t];
				char out[9];
				out[0] = (vali%10000)/1000 + 0x30;
				out[1] = (vali%1000)/100 + 0x30;
				out[2] = (vali%100)/10 + 0x30;
				out[3] = (vali%10) + 0x30;
				out[4] = '\n';
				out[5] = '\r';
				UARTWrite(out,6);
			}
			UARTWrite("\n\r",2);
		}*/
		//FIXME: End debug
		
		
		//Search for the pitch corresponding to this frequency
		uint8_t start = 0;
		uint8_t stop = NUM_PITCHES-1;
		uint8_t center = (stop + start) >> 1;
		while((stop - start) > 1)
		{
			if(pitches[center] < mainFreqf)
			{
				start = center;
			}
			else
			{
				stop = center;
			}
			center = (stop + start) >> 1;
		}
		
		//Pick the pitch with the lowest error
		float centErrf;
		/*if(abs(pitches[start] - mainFreqf) < abs(pitches[stop] - mainFreqf))
		{
			center = start;
			centErrf = (pitches[start] - mainFreqf) / cent[start];
		}
		else
		{
			center = stop;
			centErrf = (pitches[stop] - mainFreqf) / cent[stop];
		}*/
		
		{
			centErrf = 1000.0f;
			uint8_t k = 1;
			for(k = 1; k < NUM_PITCHES; k++)
			{
				if( abs(pitches[k] - mainFreqf) / cent[k] < abs(centErrf) )
				{
					centErrf = (pitches[k] - mainFreqf) / cent[k];
					center = k;
				}
			}
		}
		
		//Print the value:
		int8_t centErr  = (int8_t)(centErrf+0.5);
		
		
		//printf("Main frequency: %f\n", mainFreq);
		/*char out[9];
		out[7] = '\n';
		out[8] = '\r';
		out[4] = ' ';
		out[5] = 'H';
		out[6] = 'z';
		out[0] = (mainFreq%10000)/1000 + 0x30;
		out[1] = (mainFreq%1000)/100 + 0x30;
		out[2] = (mainFreq%100)/10 + 0x30;
		out[3] = (mainFreq%10) + 0x30;
		UARTWrite(out,9);
		
		//print out cent error, eg:
		//Ab+67 C\n\r
		out[7] = '\n';
		out[8] = '\r';
		out[5] = ' ';
		out[6] = 'C';
		out[0] = (center%100)/10 + 0x30;
		out[1] = (center%10) + 0x30;
		out[2] = centErr < 0 ? '+' : '-'; //note how centErr is calculated
		out[3] = (abs(centErr)%100)/10 + 0x30;
		out[4] = (abs(centErr)%10) + 0x30;
		UARTWrite(out,9);*/
		
		/*
		char out[8];
		out[6] = '\n';
		out[7] = '\r';
		uint16_t j = 0;
		for(j = 0; j < N_POINTS/2 + 1; j++)
		{
			int16_t real = fftout[j].r;
			int16_t imag = fftout[j].i;
			int16_t val = sqrt(real*real + imag*imag);
			out[0] = (val%1000000)/100000 + 0x30;
			out[1] = (val%100000)/10000 + 0x30;
			out[2] = (val%10000)/1000 + 0x30;
			out[3] = (val%1000)/100 + 0x30;
			out[4] = (val%100)/10 + 0x30;
			out[5] = (val%10) + 0x30;
			UARTWrite(out,8);
		}*/
		
		//Light LEDs to show sharp or flat
		LPC_GPIO0->DATA |= LED_ALL; //turn off all LEDs
		
		if(centErr <= -CENT_VFVS)
		{
			LPC_GPIO0->DATA ^= LED_VF;
		}
		else if(centErr <= -CENT_FS)
		{
			LPC_GPIO0->DATA ^= LED_F;
		}
		else if(centErr <= CENT_IT)
		{
			LPC_GPIO0->DATA ^= LED_IT;
		}
		else if(centErr <= CENT_FS)
		{
			LPC_GPIO0->DATA ^= LED_S;
		}
		else if(centErr <= CENT_VFVS)
		{
			LPC_GPIO0->DATA ^= LED_VS;
		}
		
		//Prepare for next loop
		Delay(250);
		i = 0;
		Timer3Go();
	}
}
