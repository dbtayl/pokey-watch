#ifndef PITCH_DETECT_H
#define PITCH_DETECT_H

#define LED_VF 0x00000040
#define LED_F 0x00000080
#define LED_IT 0x00000100
#define LED_S  0x00000200
#define LED_VS 0x00000400
#define LED_ALL 0x000007C0

#define CENT_VFVS 15
#define CENT_FS 10
#define CENT_IT 5

#include <inttypes.h>
#include "kiss_fftr.h"

//If the peak value detected is less than this value, no pitch-matching
//will be done and no LED will light
#define NOISE_FLOOR 8

//This value is returned in case of no pitch (noise floor not exceeded)
#define RET_NO_PITCH -1.0f

extern volatile uint16_t i;
extern volatile kiss_fft_cpx fftout[];

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
//#define SAMPLE_FREQ 1800
#define SAMPLE_FREQ 1200
#define N_POINTS 384

extern volatile kiss_fft_scalar data[];
#endif //PITCH_DETECT_FFT




#ifdef PITCH_DETECT_AUTO
#define SAMPLE_FREQ 1000
#define N_POINTS 256
extern float hanning[];
extern int16_t data[];

#endif //PITCH_DETECT_AUTO


#endif //PITCH_DETECT_H
