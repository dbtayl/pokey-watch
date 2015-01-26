//FIXME: Should add timeout to go to sleep/shutdown if no
//input for some time


#include "uafunc.h"
#include "pitches.h"
#include "pitch-detect.h"

void setup()
{	
	LPC_IOCON->SWCLK_PIO0_10 |= 0x00000001;
	LPC_GPIO0->DATA = 0xFFFFFFFF;
	LPC_GPIO0->DIR |= (1<<6)|(1<<7)|(1<<8)|(1<<9)|(1<<10);
    
	UARTInit(115200);
	ADCInit(CHN0);
	
	//NOTE: The timer interrupt DOES happen at the correct frequency
	Timer3Init(72000000/(2*SAMPLE_FREQ) - 1);
	Timer3Match0(1,INTERRUPT|RESET);//This will generate an interrupt every OTHER time the divisor is hit
	
	//Make sure we can initialize variables
	if( pitchDetectInit() )
	{
		LPC_GPIO0->DATA = 0xFFFFFFFF ^ ( LED_F | LED_S );
		while(1) {}
	}
}

#ifdef PITCH_DETECT_AUTO
void Timer3Interrupt0()
{
	data[i] = ADCRead(CHN0) * hanning[i];
	i++;
	if(i >= N_POINTS) //we really just care about ==, but >= is less error-prone
	{
		Timer3Pause();
	}
}
#else
void Timer3Interrupt0()
{
	data[i] = ADCRead(CHN0);
	i++;
	if(i >= N_POINTS) //we really just care about ==, but >= is less error-prone
	{
		Timer3Pause();
	}
}
#endif
  
void loop()
{
	//Prepare for next loop
	i = 0;
	Timer3Go();
	
	float mainFreqf = pitchDetect();
	
	//If no pitch is detected, turn off LEDs and abort this iteration
	if(mainFreqf == RET_NO_PITCH)
	{
		LPC_GPIO0->DATA |= LED_ALL; //turn off all LEDs
		return;
	}
	
	int mainFreq = (int)(mainFreqf + 0.5f);
	
	
	//FIXME: Debug: FFT results
	/*{
		int t = 0;
		for(t = 0; t < N_POINTS/2 + 1; t++)
		{
			int vali = (int)sqrt(fftout[t].r * fftout[t].r + fftout[t].i * fftout[t].i);
			char out[9];
			out[0] = (vali%10000)/1000 + 0x30;
			out[1] = (vali%1000)/100 + 0x30;
			out[2] = (vali%100)/10 + 0x30;
			out[3] = (vali%10) + 0x30;
			out[4] = '\n';
			out[5] = '\r';
			UARTWrite(out,6);
		}
	}*/
	//FIXME: End debug
	
	//FIXME: Debug: Raw data
	{
		int t = 0;
		for(t = 0; t < N_POINTS; t++)
		{
			int vali = data[t];
			char out[9];
			out[0] = (vali%10000)/1000 + 0x30;
			out[1] = (vali%1000)/100 + 0x30;
			out[2] = (vali%100)/10 + 0x30;
			out[3] = (vali%10) + 0x30;
			out[4] = '\n';
			out[5] = '\r';
			UARTWrite(out,6);
		}
	}
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
			if( abs(mainFreqf - pitches[k]) / cent[k] < abs(centErrf) )
			{
				centErrf = (mainFreqf - pitches[k]) / cent[k];
				center = k;
			}
		}
	}
	
	//Print the value:
	int8_t centErr  = (int8_t)(centErrf+0.5);
	
	
	//printf("Main frequency: %f\n", mainFreq);
	char out[9];
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
	out[2] = centErr < 0 ? '-' : '+';
	out[3] = (abs(centErr)%100)/10 + 0x30;
	out[4] = (abs(centErr)%10) + 0x30;
	UARTWrite(out,9);
	
	UARTWrite("\n\r",2);
	
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
}
