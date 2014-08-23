#ifndef PITCH_DETECT_H
#define PITCH_DETECT_H

#define LED_VF 0x00000040
#define LED_F 0x00000080
#define LED_IT 0x00000100
#define LED_S  0x00000200
#define LED_VS 0x00000400
#define LED_ALL 0x000007C0

#define CENT_VFVS 21
#define CENT_FS 14
#define CENT_IT 7

//Define PITCH_DETECT_FFT to use the FFT approach to determine pitches.
//Define PITCH_DETECT_AUTO to use autocorrelation method
//Both use the same prototypes
int pitchDetectInit();
float pitchDetect();


#ifdef PITCH_DETECT_FFT
#undef PITCH_DETECT_AUTO
#include "kiss_fftr.h"
#include "_kiss_fft_guts.h"

//NOTE: 256 points seems to be the max with 8k of RAM- ~800B short of 512...
//Works with frequencies up to 900 Hz (which is ~A5)
//FIXME: Trying lower sampling frequency- up to ~Bb4
//#define SAMPLE_FREQ 1800
#define SAMPLE_FREQ 1000
#define N_POINTS 256

extern volatile uint16_t i;
extern volatile kiss_fft_scalar data[];
#endif




#ifdef PITCH_DETECT_AUTO

#endif

#endif
