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
    
    //FIXME: Disable UART for final version
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
	//Also ignore if the pitch detected is outside of our pitch detection bounds
	if( (mainFreqf == RET_NO_PITCH) || (mainFreqf < pitches[0] - 100*cent[0]) || (mainFreqf > pitches[NUM_PITCHES-1] + 100*cent[NUM_PITCHES-1]) )
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
	/*{
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
	}*/
	//FIXME: End debug
	
	//Find the note associated with the pitch detected
	uint8_t center;
	float centErrf = 1000.0f;
	{
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
	
	//Integer version of cents sharp/flat
	int8_t centErr  = (int8_t)(centErrf+0.5);
	int8_t absCentErr = abs(centErr);
	
	//FIXME: Remove for release
	//Print out detected frequency
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
	out[3] = (absCentErr%100)/10 + 0x30;
	out[4] = (absCentErr%10) + 0x30;
	UARTWrite(out,9);
	
	UARTWrite("\n\r",2);
	
	
	//Light LEDs to show sharp or flat
	LPC_GPIO0->DATA |= LED_ALL; //turn off all LEDs
	
	if(absCentErr <= CENT_IT)
	{
		LPC_GPIO0->DATA ^= LED_IT;
	}
	else if(absCentErr <= CENT_FS)
	{
		if(centErr < 0)
		{
			LPC_GPIO0->DATA ^= LED_F;
		}
		else
		{
			LPC_GPIO0->DATA ^= LED_S;
		}
	}
	else
	{
		if(centErr < 0)
		{
			LPC_GPIO0->DATA ^= LED_VF;
		}
		else
		{
			LPC_GPIO0->DATA ^= LED_VS;
		}
	}
}
