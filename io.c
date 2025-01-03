/*
 * io.c
 *
 * Created: 15/07/2023
 * Author : Richard Tomlinson G4TGJ
 */ 

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pico/multicore.h"

#include "config.h"
#include "io.h"
#include "fir.h"
#include "hilbert.h"
#include "sdr.h"

// Map between relay number and the output GPIO
static const uint8_t  relayGpio[NUM_RELAYS] =
{
#if NUM_RELAYS > 0
    RELAY_0_OUTPUT_PIN,
#endif
#if NUM_RELAYS > 1
    RELAY_1_OUTPUT_PIN,
#endif
#if NUM_RELAYS > 2
    RELAY_2_OUTPUT_PIN,
#endif
#if NUM_RELAYS > 3
    RELAY_3_OUTPUT_PIN,
#endif
#if NUM_RELAYS > 4
    RELAY_4_OUTPUT_PIN,
#endif
};

// Table of volume multipliers from 0 to max
// They are logarithmic
#define VOLUME_PRECISION 256
static int volumeMultiplier[MAX_VOLUME + 1] =
{
    0,
    1,
    3,
    5,
    7,
    9,
    11,
    14,
    18,
    23,
    29,
    36,
    46,
    57,
    72,
    91,
    114,
    144,
    181,
    228,
    287,
    362,
    455,
    573,
    722,
    908,
    1144,
    1440,
    1812,
    2282,
    2872,
    3616,
    4552,
    5108,
    5731,
    6430,
    7215,
    8095,
    9083,
    10192,
    11435,
};

// Approx 700Hz sine wave
#define NUM_SINE700 80
static const int sine700[NUM_SINE700] =
{
0,
535,
912,
1021,
828,
392,
-160,
-665,
-974,
-996,
-724,
-239,
316,
779,
1011,
946,
602,
80,
-465,
-873,
-1024,
-873,
-465,
80,
602,
946,
1011,
779,
316,
-239,
-724,
-996,
-974,
-665,
-160,
392,
828,
1021,
912,
535,
0,
-535,
-912,
-1021,
-828,
-392,
160,
665,
974,
996,
724,
239,
-316,
-779,
-1011,
-946,
-602,
-80,
465,
873,
1024,
873,
465,
-80,
-602,
-946,
-1011,
-779,
-316,
239,
724,
996,
974,
665,
160,
-392,
-828,
-1021,
-912,
-535,
};

#define ADC_BUFFER_SIZE 32
#define INPUT_SIZE ADC_BUFFER_SIZE

static int captureDma;
static dma_channel_config captureCfg;

static int adcDma0;
static int adcDma1;
static dma_channel_config dmaCfg0;
static dma_channel_config dmaCfg1;
static uint16_t adcBuffer0[ADC_BUFFER_SIZE];
static uint16_t adcBuffer1[ADC_BUFFER_SIZE];

// Keep track of whether the input has overloaded
// Unsigned so overloads if goes near zero or max
#define ADC_OVERLOAD_LOW 10
#define ADC_OVERLOAD_HIGH 4080

// Set true if overload detected
bool adcOverload;

// Keep track of overload on the output
//  0=no overflow
//  1=too large
// -1=too small
int outOverload;

// Reset the overload flag every second
#define ADC_OVERLOAD_RESET_COUNT 8000
static int adcOverloadResetCount;

#ifdef DISPLAY_MIN_MAX
// Maximum input and output
int maxIn, maxOut, minIn, minOut;
#endif

// Maximum mute factor when unmuting
int maxMuteFactor = DEFAULT_MAX_MUTE_FACTOR;

// Select normal, binaural or peaked output
enum eOutput outputMode;

// Generate a sinewave for the sidetone
static int currentSinePos;
static inline int sinewave()
{
    int val = sine700[currentSinePos];
    currentSinePos = (currentSinePos+1) % NUM_SINE700;

    return val;
}

// Functions to read and write inputs and outputs
// This isolates the main logic from the I/O functions making it
// easy to change the hardware e.g. we have the option to
// connect several inputs to a single ADC pin

static void gpioSetOutput( uint16_t gpio )
{
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
}

static void gpioSetInputWithPullup( uint16_t gpio )
{
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_IN);
    gpio_pull_up(gpio);
}

// The ADC reads between 0 and 4095 but the DSP needs it to be signed
// so subract this from samples and add back before writing to PWM
#define ADC_OFFSET 2048

#define MAX_PWM_OUT 4095

// PWM slices for sidetone and audio outputs
static uint16_t sidetone_slice_num;
static uint16_t audio_l_slice_num;
static uint16_t audio_r_slice_num;

static int iOutBuffer[OUTPUT_BUFFER_LEN];
static int qOutBuffer[OUTPUT_BUFFER_LEN];
static int hilbertBuffer[HILBERT_FILTER_BUFFER_LEN];
static int delayBuffer[HILBERT_FILTER_BUFFER_LEN];
static int leftBuffer[LEFT_BUFFER_LEN];
static int rightBuffer[RIGHT_BUFFER_LEN];
static int outBuffer[OUTPUT_BUFFER_LEN];

static uint16_t iOutCurrentSample;
static uint16_t qOutCurrentSample;
static uint16_t hilbertCurrentSample;
static uint16_t delayCurrentSample;
static uint16_t leftCurrentSample;
static uint16_t rightCurrentSample;
static uint16_t outCurrentSample;

// Input and output positions for 128/64k filter
// Input and output are tracked separately as the
// input is filled at the start of the interrupt handler
static uint16_t iInPos, qInPos;
static uint16_t iIn12864, qIn12864;
static uint16_t iOut12864, qOut12864;

// Positions for the other filters
// Input and output track each other
static uint16_t iPos6432, qPos6432;
static uint16_t iPos3216, qPos3216;
static uint16_t iPos1608, qPos1608;

static uint16_t iInInterpolate, iOutInterpolate;
static uint16_t qInInterpolate, qOutInterpolate;

static uint16_t iIn6432, iOut6432;
static uint16_t qIn6432, qOut6432;

static uint16_t iIn3216, iOut3216;
static uint16_t qIn3216, qOut3216;

static uint16_t iIn1608, iOut1608;
static uint16_t qIn1608, qOut1608;

static uint16_t iOut08, qOut08;

static int         iInputBuffer[DECIMATE_BUFFER_LEN];
static int         qInputBuffer[DECIMATE_BUFFER_LEN];

static int    iAccumulateBuffer[DECIMATE_BUFFER_LEN];
static int    qAccumulateBuffer[DECIMATE_BUFFER_LEN];

static int iDecimate12864Buffer[DECIMATE_BUFFER_LEN];
static int qDecimate12864Buffer[DECIMATE_BUFFER_LEN];
static int  iDecimate6432Buffer[DECIMATE_BUFFER_LEN];
static int  qDecimate6432Buffer[DECIMATE_BUFFER_LEN];
static int  iDecimate3216Buffer[DECIMATE_BUFFER_LEN];
static int  qDecimate3216Buffer[DECIMATE_BUFFER_LEN];
static int  iDecimate1608Buffer[DECIMATE_BUFFER_LEN];
static int  qDecimate1608Buffer[DECIMATE_BUFFER_LEN];

// The following can be configured in menus so are global
enum sdrSource outputSource;
static int currentFilter = DEFAULT_FILTER;
static int currentHilbertFilter = DEFAULT_HILBERT_FILTER;
bool applyRoofingFilter = true;

// Whether or not intermediate frequency is above or below
bool ifBelow = true;

int iGain = DEFAULT_I_GAIN;
int qGain = DEFAULT_Q_GAIN;
int iqGain = DEFAULT_IQ_GAIN;

bool applyGains;
bool adjustPhase;

volatile uint32_t maxInput;

static uint8_t pwmDivider = AUDIO_DIVIDE;

static uint32_t idleCount;
uint32_t currentCount;

// Shift the input to provide some initial gain and
// not lose too much precision in filter calculations
int inputShift = DEFAULT_INPUT_SHIFT;

// Main and sidetone volumes (indexes into the multiplier table)
static uint8_t volume = DEFAULT_VOLUME;
static uint8_t sidetoneVolume = DEFAULT_SIDETONE_VOLUME;

static bool sidetoneOn;

static bool bMuteRX;

void ioClearScale( void )
{
    maxInput = 0;
}

uint32_t ioGetScale( void )
{
#if 0
    return maxInput;
#else
    int scale = 0;

    if( maxInput > 0x200 )
    {
        scale = 5;
    }
    else if( maxInput > 0x100 )
    {
        scale = 4;
    }
    else if( maxInput > 0x80 )
    {
        scale = 3;
    }
    else if( maxInput > 0x20 )
    {
        scale = 2;
    }
    else if( maxInput > 0x1 )
    {
        scale = 1;
    }
    return scale;
#endif
}

static inline int delay( int sample, uint16_t *current, int *buffer, int bufLen, int delay )
{
    int result = 0;

    // Store the delayed value
    result = buffer[(*current-delay) & (bufLen-1)];

    // Store the new sample
    buffer[*current] = sample;

    // Move to the next sample position, wrapping as needed
    *current = (*current+1) & (bufLen-1);

    return buffer[(*current-delay) & (bufLen-1)];
}

static inline void firIn( int sample, uint16_t *current, int *buffer, int bufLen )
{
    // Store the new sample
    buffer[*current] = sample;

    // Move to the next sample position, wrapping as needed
    *current = (*current+1) & (bufLen-1);
}

// bufLen must be a power of 2
static inline int firOut( uint16_t current, int *buffer, int bufLen, const int *taps, int numTaps, int precision )
{
    int tap;
    uint16_t index;
    int32_t result = 0;

    // Calculate the result from the FIR filter
    index = current;
    for( tap = 0 ; tap < numTaps ; tap++ )
    {
        index = (index-1) & (bufLen-1);
        result += ((int32_t) buffer[index]) * taps[tap];
    }

    // Extract the significant bits
    //return result  / 32768; //>> precision;
    return (result + 16384) / 32768; //>> precision;
    //return (result + 32768) / 65536;
}

static int inline fir( int sample, uint16_t *current, int *buffer, int bufLen, const int *taps, int numTaps, int precision )
{
    firIn( sample,  current, buffer, bufLen );
    return firOut( *current, buffer, bufLen, taps, numTaps, precision );
}

// Filter and decimate
//
// factor - Decimation factor
// count - Number of samples to process
// inBuf - Pointer to input buffer containing the samples
// inBufLen - Length of the input buffer (Must be power of 2)
// inPos - Pointer to position in the input buffer
// outBuf - Pointer to output buffer
// outBufLen - Length of the output buffer (Must be power of 2)
// taps - Pointer to FIR filter taps
// numTaps - The number of FIR taps
// precision - Bits of precision of the FIR
//
// count/factor must be an integer
static inline void decimate( int factor, int count, int *inBuf, int inBufLen, uint16_t *inPos, int *outBuf, int outBufLen, uint16_t *outPos, const int *taps, int numTaps, int precision )
{
    // Process each decimation
    for( int i = 0 ; i < count/factor ; i++ )
    {
        int out = firOut( *inPos, inBuf, inBufLen, taps, numTaps, precision );
        firIn( out, outPos, outBuf, outBufLen );

        // Move the input pointer past the decimation factor
        *inPos = (*inPos+factor) & (inBufLen-1);
    }
}

static inline void decimateCIC( int factor, int order, int gainBits, int count, int *inBuf, int inBufLen, uint16_t *inPos, int *accumulator, int *combOutput, int *previousInput, int *outBuf, int outBufLen, uint16_t *outPos )
{
    int i,j;

    // Process each sample
    for( i = 0 ; i < count ; i++ )
    {
        int in = inBuf[*inPos];

        // Integrate
        accumulator[0] += in;
        for( j = 1 ; j < order ; j++ )
        {
            accumulator[j] += accumulator[j-1];
        }

        // Move the input pointer
        *inPos = (*inPos+1) & (inBufLen-1);

        // Decimate
        if( (i % factor) == 0 )
        {
            // Input to the comb is the final accumulator value
            // from the integrator.
            // j-1 is because the for has incremented at the end.
            int input = accumulator[j-1];

            // Comb
            combOutput[0] = input - previousInput[0];
            previousInput[0] = input;
            for( j = 1 ; j < order ; j++ )
            {
                // Input to the stage is the output of the previous stage
                input = combOutput[j-1];

                // Output is the current input minus the previous input to this stage
                combOutput[j] = input - previousInput[j];

                // Store the input for next time
                previousInput[j] = input;
            }

            // Store the result in the output buffer
            // j-1 is because the for has incremented at the end.
            outBuf[*outPos] = combOutput[j-1] >> gainBits;

            // Move the output pointer
            *outPos = (*outPos + 1) & (outBufLen-1);
        }
    }
}

// bufLen must be a power of 2
static int inline hilbert( int sample, uint16_t *current, int *buffer, int bufLen, const int *taps, int numTaps, int precision )
{
    int tap;
    uint16_t index;
    int32_t result = 0;

    // Store sample in the FIR buffer
    firIn( sample, current, buffer, bufLen );

    // Calculate the result from the FIR filter
    index = *current;
//    for( tap = numTaps-1 ; tap >= 0 ; tap-- )
    for( tap = 0 ; tap < numTaps ; tap++ )
    {
        index = (index-1) & (bufLen-1);
        result += ((int32_t) buffer[index]) * taps[tap];
    }

    // Extract the significant bits
    return result >> precision;
}

// CW audio peaking filter
#define A0  32768
#define A1 -55751
#define A2  32619

#define B0  32992
#define B1 -55751
#define B2  32394

static int inline cwPeakIIR( int x )
{
    static int xm1, xm2, ym1, ym2;
    int y = (((B0 * x) + (B1 * xm1) + (B2 * xm2) - (A1 * ym1) - (A2 * ym2)) + (A0/2)) / A0;

    xm2 = xm1;
    xm1 = x;
    ym2 = ym1;
    ym1 = y;

    return y;
}

static bool adcDebugState1;

static inline void toggleAdcDebug1()
{
#ifdef ADC_DEBUG1_OUTPUT_GPIO
    adcDebugState1 = !adcDebugState1;
    gpio_put(ADC_DEBUG1_OUTPUT_GPIO, adcDebugState1);
#endif
}

static inline void setAdcDebug1()
{
#ifdef ADC_DEBUG1_OUTPUT_GPIO
    adcDebugState1 = true;
    gpio_put(ADC_DEBUG1_OUTPUT_GPIO, adcDebugState1);
#endif
}

static inline void clearAdcDebug1()
{
#ifdef ADC_DEBUG1_OUTPUT_GPIO
    adcDebugState1 = false;
    gpio_put(ADC_DEBUG1_OUTPUT_GPIO, adcDebugState1);
#endif
}

static bool adcDebugState2;

static inline void toggleAdcDebug2()
{
#ifdef ADC_DEBUG2_OUTPUT_GPIO
    adcDebugState2 = !adcDebugState2;
    gpio_put(ADC_DEBUG2_OUTPUT_GPIO, adcDebugState2);
#endif
}

static inline void setAdcDebug2()
{
#ifdef ADC_DEBUG2_OUTPUT_GPIO
    adcDebugState2 = true;
    gpio_put(ADC_DEBUG2_OUTPUT_GPIO, adcDebugState2);
#endif
}

static inline void clearAdcDebug2()
{
#ifdef ADC_DEBUG2_OUTPUT_GPIO
    adcDebugState2 = false;
    gpio_put(ADC_DEBUG2_OUTPUT_GPIO, adcDebugState2);
#endif
}

// The next outputs to send to the PWM
static int outPWMLeft;
static int outPWMRight;

// Calculate the output PWM value
static inline int calcPWM( int out )
{
    // Convert to unsigned
    int outPWM = out + ADC_OFFSET;

    // Don't let the PWM output go too low or too high
    if( outPWM < 0 )
    {
        outOverload = -1;
        outPWM = 0;
    }
    else if( outPWM > MAX_PWM_OUT )
    {
        outOverload = 1;
        outPWM = MAX_PWM_OUT;
    }

    return outPWM;
}

// Numerically controlled oscillator

// Integer value representing 1.0
#define NCO_SCALE   32768

// The number of bits to shift the phase down
#define NCO_BITS    3

// Length of the cosine table
#define NCO_COS_LEN 8192

// Cosine table. Populated in ncoInit()
static int ncoCos[NCO_COS_LEN];

struct sNCO
{
    // The frequency control word. Calculated by ncoSet()
    uint16_t ncoFCW;

    // The current phase
    uint16_t phase;
};

// Oscillator for the IF
static struct sNCO ncoIF;

// Oscillator for the BFO
static struct sNCO ncoBFO;

// Calculates the frequency control word for the frequency and given sample rate
static void ncoSet( struct sNCO *nco, int frequency, int sampleRate)
{
    nco->phase = 0;
    nco->ncoFCW = ((frequency * NCO_COS_LEN) << NCO_BITS) / sampleRate;
}

// Returns the next value of the oscillator
// Use this or use ncoOscI(), ncoOscQ() and ncoOscIncrementPhase()
static inline int ncoOsc( struct sNCO *nco )
{
    int result = ncoCos[nco->phase >> NCO_BITS];
    nco->phase += nco->ncoFCW;
    return result;
}

// Returns the I component of the oscillator.
// Must be followed by ncoOscIncrementPhase()
static inline int ncoOscI( struct sNCO *nco )
{
    return ncoCos[nco->phase >> NCO_BITS];
}

// Returns the Q component of the oscillator.
// Must be followed by ncoOscIncrementPhase()
static inline int ncoOscQ( struct sNCO *nco )
{
    return ncoCos[((nco->phase >> NCO_BITS) - NCO_COS_LEN/4)&(NCO_COS_LEN-1)];
}

// Moves to the next phase of the oscillator.
// Use after calling ncoOscI() and/or ncoOscQ().
static inline void ncoOscIncrementPhase( struct sNCO *nco )
{
    nco->phase += nco->ncoFCW;
}

// Initialise the NCO cosine table
static void ncoInit( void )
{
    for( int i = 0 ; i < NCO_COS_LEN ; i++ )
    {
        ncoCos[i] = cos( 2.0 * M_PI * i / NCO_COS_LEN) * NCO_SCALE;
    }
}

static inline void complexMixer( int *pI, int *pQ, struct sNCO *ncoIF )
{
    // Get the next quadrature oscillator value
    int iOsc = ncoOscI(ncoIF);
    int qOsc = ncoOscQ(ncoIF);
    ncoOscIncrementPhase(ncoIF);

    // Multiply to get the frequency shift
    int i = (((*pI * iOsc) - (*pQ * qOsc))  + 16384) / 32768; // / NCO_SCALE;
    int q = (((*pI * qOsc) + (*pQ * iOsc))  + 16384) / 32768; // / NCO_SCALE;

    // Store the new values
    *pI = i;
    *pQ = q;
}

// Remove DC. Requires two static values to keep track.
static inline int removeDC( int x, int *xm1, int *ym1 )
{
    int y = x - *xm1 + (*ym1 * 254) / 256;
    *xm1 = x;
    *ym1 = y;

    return y;
}

// The gain representing unity i.e. no AGC action
// Increasing this value will slow down the AGC but to
// keep fast attack can increase GAIN_DECREASE_STEP
// Also need to be aware of possible overload
#define AGC_UNITY_GAIN 8192

// The threshold above which we reduce the gain
// Features the buffer length as the more samples we store the higher the
// amplitude value will be.
#define AGC_THRESHOLD (1024 * AGC_BUFFER_LEN * AGC_BUFFER_LEN)

// We reduce the gain in steps. If we sample at 8ksps and reduce the gain
// by, say, 800 one at a time that will take 100ms so decrease by more to reduce the
// time it takes.
// The time taken will depend on the value of AGC_UNITY_GAIN.
#define GAIN_DECREASE_STEP 10

// We store samples in order to calculate the amplitude
static int agcBuffer[AGC_BUFFER_LEN];

// Position in the buffer
static uint16_t agcPos;

// The amplitude calculated over a number of samples
uint32_t agcAmplitude;

// The gain set by AGC
int agcGain = AGC_UNITY_GAIN;

// Applies AGC to the sample
static inline int applyAGC( int in )
{
    // Calculate the amplitude of the incoming sample. No need to take the square root.
    int inAmplitude = in * in;

    // Calculate the amplitude over the samples
    agcAmplitude = agcAmplitude - agcBuffer[agcPos] + inAmplitude;

    // Feed the input amplitude into the buffer
    firIn( inAmplitude, &agcPos, agcBuffer, AGC_BUFFER_LEN);

    // If the amplitude is higher than the threshold then reduce the gain
    // Lower the gain more for even louder signals
    // Do this gradually
    if( agcAmplitude > 4 * AGC_THRESHOLD )
    {
        if( agcGain > AGC_UNITY_GAIN / 8 )
        {
            agcGain -= GAIN_DECREASE_STEP;
        }
    }
    else if( agcAmplitude > 2 * AGC_THRESHOLD )
    {
        if( agcGain > AGC_UNITY_GAIN / 4 )
        {
            agcGain -= GAIN_DECREASE_STEP;
        }
    }
    else if( agcAmplitude > AGC_THRESHOLD )
    {
        if( agcGain > AGC_UNITY_GAIN / 2 )
        {
            agcGain -= GAIN_DECREASE_STEP;
        }
    }
    else
    {
        // Amplitude is below the threshold so increase the
        // gain back towards unity
        if( agcGain < AGC_UNITY_GAIN )
        {
            agcGain++;
        }
    }

    // Apply the gain
    return (in * agcGain) / AGC_UNITY_GAIN;
}

// Process the input I and Q samples and generate the output sample
// The input and the output are done in the interrupt handler
// but the processing is done in the main loop
static inline void processAudio()
{
    int out, outI, outQ, inI, inQ, outLeft, outRight, i32, q32, i64, q64;
    static int aveI, aveQ;

    // Used while removing DC
    static int iXm1, iYm1;
    static int qXm1, qYm1;

    // The volume, either normal or sidetone, to apply to the output
    int vol;

    if( applyRoofingFilter )
    {
        setAdcDebug2();
        // Shift the frequency down 8kHz
        for( int p = 0 ; p < INPUT_SIZE ; p++ )
        {
            // Get the input buffer positions allowing for buffer wrap
            int iPos = (iIn12864+p)&(DECIMATE_BUFFER_LEN-1);
            int qPos = (qIn12864+p)&(DECIMATE_BUFFER_LEN-1);
            
            complexMixer( &iInputBuffer[iPos], &qInputBuffer[qPos], &ncoIF );
        }
        clearAdcDebug2();

#define CIC_ORDER 3
#define CIC_DECIMATION_FACTOR 16

// Number of bits to shift the CIC result right
// 2^CIC_GAIN_BITS is the gain of the filter at DC
// Equal to log2(CIC_DECIMATION_FACTOR^CIC_ORDER) = CIC_ORDER*log2(CIC_DECIMATION_FACTOR)
// Subtract 1 to allow for interpolation with zero
// Assuming 12 bit samples and 32 bit words maximum is 20 bits before overflow
#define CIC_GAIN_BITS 11

        static int iCombOutput[CIC_ORDER];
        static int qCombOutput[CIC_ORDER];
        static int iCombPreviousInput[CIC_ORDER];
        static int qCombPreviousInput[CIC_ORDER];

        decimateCIC( CIC_DECIMATION_FACTOR, CIC_ORDER, CIC_GAIN_BITS, INPUT_SIZE, iInputBuffer, DECIMATE_BUFFER_LEN, &iIn12864, iAccumulateBuffer, iCombOutput, iCombPreviousInput, iDecimate1608Buffer, DECIMATE_BUFFER_LEN, &iOut12864 );
        decimateCIC( CIC_DECIMATION_FACTOR, CIC_ORDER, CIC_GAIN_BITS, INPUT_SIZE, qInputBuffer, DECIMATE_BUFFER_LEN, &qIn12864, qAccumulateBuffer, qCombOutput, qCombPreviousInput, qDecimate1608Buffer, DECIMATE_BUFFER_LEN, &qOut12864 );

        // Filter and decimate from 16ksps to 8ksps
        // This leaves us with a single pair of I and Q
        decimate( 2, INPUT_SIZE/CIC_DECIMATION_FACTOR, iDecimate1608Buffer, DECIMATE_BUFFER_LEN, &iIn1608, &inI, 1, &iOut08, decimate1608FilterTaps, DECIMATE_16_08_FILTER_TAP_NUM, DECIMATE_16_08_FILTER_PRECISION );
        decimate( 2, INPUT_SIZE/CIC_DECIMATION_FACTOR, qDecimate1608Buffer, DECIMATE_BUFFER_LEN, &qIn1608, &inQ, 1, &qOut08, decimate1608FilterTaps, DECIMATE_16_08_FILTER_TAP_NUM, DECIMATE_16_08_FILTER_PRECISION );
    }
    else
    {
        inI = iInputBuffer[iInPos];
        inQ = qInputBuffer[qInPos];
    }

    // Apply I and Q gains
    if( applyGains )
    {
        inI = ((int)inI) * iGain / 32768;
        inQ = ((int)inQ) * qGain / 32768;
    }

    // Adjust phase by adding some Q to I
    if( adjustPhase )
    {
        inI += ((int)inQ) * iqGain / 32768;
    }

    // Apply the CW filter
    if( currentFilter >= 0 && currentFilter < NUM_FILTERS && filters[currentFilter].taps != NULL)
    {
        outI = fir(inI, &iOutCurrentSample, iOutBuffer, OUTPUT_BUFFER_LEN, filters[currentFilter].taps, filters[currentFilter].numTaps, filters[currentFilter].precision);
        outQ = fir(inQ, &qOutCurrentSample, qOutBuffer, OUTPUT_BUFFER_LEN, filters[currentFilter].taps, filters[currentFilter].numTaps, filters[currentFilter].precision);
    }
    else
    {
        outI = inI;
        outQ = inQ;
    }

    switch( outputSource )
    {
        case sdrIPQ:
        case sdrIMQ:
#if 1
            // Apply the BFO to get the CW tone where we want it
            complexMixer( &outI, &outQ, &ncoBFO );

            // Only need I now
            out = outI;
#else
            // Apply the hilbert filter to I and delay Q to match
            if( currentHilbertFilter == 0 )
            {
                outI = inI;
                outQ = inQ;
            }
            else
            {
                outI = hilbert(inI, &hilbertCurrentSample, hilbertBuffer, HILBERT_FILTER_BUFFER_LEN, hilbertFilters[currentHilbertFilter].taps, hilbertFilters[currentHilbertFilter].numTaps, hilbertFilters[currentHilbertFilter].precision);
                outQ = delay(inQ, &delayCurrentSample, delayBuffer, HILBERT_FILTER_BUFFER_LEN, (hilbertFilters[currentHilbertFilter].numTaps-1)/2);
            }
            if( outputSource == sdrIMQ )
            {
                out = (outI - outQ) / 2;
            }
            else
            {
                out = (outI + outQ) / 2;
            }
#endif
            break;
        case sdrI:
            out = inI;
            break;
        case sdrQ:
            out = inQ;
            break;
        case sdrIDelayed:
            out = delay(inI, &delayCurrentSample, delayBuffer, HILBERT_FILTER_BUFFER_LEN, (hilbertFilters[currentHilbertFilter].numTaps-1)/2);
            break;
        case sdrQDelayed:
            out = delay(inQ, &delayCurrentSample, delayBuffer, HILBERT_FILTER_BUFFER_LEN, (hilbertFilters[currentHilbertFilter].numTaps-1)/2);
            break;
        case sdrIHilbert:
            if( currentHilbertFilter == 0 )
            {
                out = inI;
            }
            else
            {
                out = hilbert(inI, &hilbertCurrentSample, hilbertBuffer, HILBERT_FILTER_BUFFER_LEN, hilbertFilters[currentHilbertFilter].taps, hilbertFilters[currentHilbertFilter].numTaps, hilbertFilters[currentHilbertFilter].precision);
            }
            break;
        case sdrQHilbert:
            if( currentHilbertFilter == 0 )
            {
                out = inQ;
            }
            else
            {
                out = hilbert(inQ, &hilbertCurrentSample, hilbertBuffer, HILBERT_FILTER_BUFFER_LEN, hilbertFilters[currentHilbertFilter].taps, hilbertFilters[currentHilbertFilter].numTaps, hilbertFilters[currentHilbertFilter].precision);
            }
            break;
        case sdrSineWave:
            out = sinewave();
            break;
        case sdrSilence:
            out = 0;
            break;
        default:
            break;
    }

#if 1
    static bool bWasMuted;

    // Mute the RX if required
    static int muteFactor;
    static bool waitingZeroCross;

    // Keep track of whether the current and previous samples are
    // positive or not - the top bit of the 32 bit integer is the sign bit
    static int prevOut;
    bool outPositive = (((int)out) & 0x80000000 ? false : true);
    bool prevOutPositive = (((int)prevOut) & 0x80000000 ? false : true);
    prevOut = out;

    if( bMuteRX )
    {
        bWasMuted = true;
        //out = 0;
        muteFactor = 0;

        // We will unmute on a zero crossing
        waitingZeroCross = true;
    }
    else
    {
        // Keep waiting for a zero cross until we see a sign change
        if( waitingZeroCross )
        {
            if( outPositive != prevOutPositive )
            {
                waitingZeroCross = false;
            }
        }

        // If we aren't waiting for a zero cross we can
        // proceed
        if( !waitingZeroCross )
        {
            // On unmute bring the volume up slowly to avoid clicks
            if( bWasMuted && (muteFactor < maxMuteFactor) )
            {
                muteFactor++;
            }
            else
            {
                bWasMuted = false;
                muteFactor = maxMuteFactor;
            }
        }
    }

    // Insert sidetone and set the appropriate volume
    // When the sidetone is switched off we fade it down to
    // reduce clicks
    static uint8_t actualSidetoneVolume;
    static uint8_t fadeRate;
    static uint8_t fadeCount;
    if( sidetoneOn )
    {
        actualSidetoneVolume = sidetoneVolume;
        fadeRate = MAX_VOLUME / sidetoneVolume + 2;
        fadeCount = 0;
    }
    else
    {
        //actualSidetoneVolume = 0;
        // Lower sidetone volumes fade down too quickly so
        // we slow them down
        fadeCount++;
        if( (fadeCount % fadeRate) == 0 )
        {
            if( actualSidetoneVolume > 0 )
            {
                actualSidetoneVolume--;
            }
        }
    }
    out = ((int)out * muteFactor) / maxMuteFactor;
#endif

#if 1
    // Apply AGC
    out = applyAGC( out );

    // Apply the volume
    out = (out*volumeMultiplier[volume])/VOLUME_PRECISION;

    // Select binaural, peaked or normal output
    switch( outputMode )
    {
        case BINAURAL_OUTPUT:
            // Apply LPF to left and HPF to right to get binaural effect
            outLeft  = fir(out, &leftCurrentSample,  leftBuffer,  LEFT_BUFFER_LEN,  leftFilterTaps,  LEFT_FILTER_TAP_NUM,  LEFT_FILTER_PRECISION);
            outRight = fir(out, &rightCurrentSample, rightBuffer, RIGHT_BUFFER_LEN, rightFilterTaps, RIGHT_FILTER_TAP_NUM, RIGHT_FILTER_PRECISION);
            break;

        case PEAKED_OUTPUT:
            outLeft = outRight = cwPeakIIR(out);
            break;

        default:
            outLeft = outRight = out;
            break;
    }
#else
    // Apply AGC
    outLeft = applyAGC( out );
    outRight = out;

    // Apply the volume
    outLeft  = (outLeft*volumeMultiplier[volume])/VOLUME_PRECISION;
    outRight = (outRight*volumeMultiplier[volume])/VOLUME_PRECISION;
#endif

    if( actualSidetoneVolume > 0 )
    {
        int sidetone = (sinewave()*volumeMultiplier[actualSidetoneVolume])/VOLUME_PRECISION;
        outLeft += sidetone;
        outRight += sidetone;
    }

    //out = fir(out, &outCurrentSample, outBuffer, OUTPUT_BUFFER_LEN, cwFilterTaps1, CW_FILTER_TAP_NUM_1, CW_FILTER_PRECISION_1);

    // Convert the samples for the PWM
    outPWMRight = calcPWM( outRight);
    outPWMLeft = calcPWM( outLeft);

#define FACTOR 128
    maxInput = (maxInput*(FACTOR-1) + abs(out))/FACTOR;
//    maxInput += abs(out)*1024;

    // Reset the ADC and output overload flags every second
    adcOverloadResetCount++;
    if( adcOverloadResetCount > ADC_OVERLOAD_RESET_COUNT )
    {
        adcOverload = false;
        adcOverloadResetCount = 0;
        outOverload = 0;
#ifdef DISPLAY_MIN_MAX
        maxOut = 0;
        maxIn = 0;
        minOut = minIn = 0xffff;
#endif
    }
}

// Feed from a DMA buffer into the FIR
static inline void feedFir( uint16_t *buf )
{
    static int ixm1, iym1;
    static int qxm1, qym1;

    for( int i = 0 ; i < ADC_BUFFER_SIZE/2 ; i++ )
    {
        // Samples alternate between I and Q in DMA buffer
        // Remove any DC
        // Zero pad
        int ix = buf[i*2];
        int iy = removeDC( ix, &ixm1, &iym1 );
        firIn(iy<<inputShift, &iInPos, iInputBuffer, DECIMATE_BUFFER_LEN);
        firIn(0,               &iInPos, iInputBuffer, DECIMATE_BUFFER_LEN);

        int qx = buf[i*2+1];
        int qy = removeDC( qx, &qxm1, &qym1 );
        firIn(0,               &qInPos, qInputBuffer, DECIMATE_BUFFER_LEN);
        firIn(qy<<inputShift, &qInPos, qInputBuffer, DECIMATE_BUFFER_LEN);

#ifdef DISPLAY_MIN_MAX
        if( ix > maxIn )
        {
            maxIn = ix;
        }

        if( ix < minIn )
        {
            minIn = ix;
        }
#endif
        if( ix < ADC_OVERLOAD_LOW || ix > ADC_OVERLOAD_HIGH || qx < ADC_OVERLOAD_LOW || qx > ADC_OVERLOAD_HIGH )
        {
            adcOverload = true;
        }
    }
}

static void adc_interrupt_handler()
{
    setAdcDebug1();

    // Write the output value determined last time. This adds a slight delay but
    // ensures that the output changes at a consistent time not dependent on the time
    // taken to do all the calculations for the filters.
    pwm_set_chan_level(audio_l_slice_num, PWM_CHAN_A, outPWMLeft);
    pwm_set_chan_level(audio_r_slice_num, PWM_CHAN_A, outPWMRight);
    
#ifdef DISPLAY_MIN_MAX
    if( outPWMLeft > maxOut )
    {
        maxOut = outPWMLeft;
    }

    if( outPWMLeft < minOut )
    {
        minOut = outPWMLeft;
    }
#endif

    // If DMA into buffer 0 completed then feed into the FIR
    if(dma_hw->ints0 & (1u << adcDma0))
    {
        dma_channel_configure(adcDma0, &dmaCfg0, adcBuffer0, &adc_hw->fifo, ADC_BUFFER_SIZE, false);
        feedFir( adcBuffer0 );
        dma_hw->ints0 = 1u << adcDma0;
    }

    // If DMA into buffer 1 completed then feed into the FIR
    if(dma_hw->ints0 & (1u << adcDma1))
    {
        dma_channel_configure(adcDma1, &dmaCfg1, adcBuffer1, &adc_hw->fifo, ADC_BUFFER_SIZE, false);
        feedFir( adcBuffer1 );
        dma_hw->ints0 = 1u << adcDma1;
    }

    processAudio();

    clearAdcDebug1();
}

void ioSetIF( void )
{
    // Set the IF oscillator
    // Because we zero stuff the sample rate
    // is twice the ADC rate
    ncoSet( &ncoIF, ifBelow ? INTERMEDIATE_FREQUENCY : -INTERMEDIATE_FREQUENCY, 2 * ADC_SAMPLE_RATE );
}

void core1_main()
{
#ifdef ADC_DEBUG1_OUTPUT_GPIO
    gpioSetOutput(ADC_DEBUG1_OUTPUT_GPIO);
#endif
#ifdef ADC_DEBUG2_OUTPUT_GPIO
    gpioSetOutput(ADC_DEBUG2_OUTPUT_GPIO);
#endif

    // Initialise the NCO
    ncoInit();

    // Set the IF oscillator
    ioSetIF();

    // Set the BFO
    ncoSet( &ncoBFO, -RX_OFFSET, BFO_SAMPLE_RATE );

    // We need the ADC for audio input
    adc_init();

    // Set up the ADC for the audio input
    adc_gpio_init(AUDIO_I_GPIO);
    adc_gpio_init(AUDIO_Q_GPIO);
    adc_set_clkdiv(ADC_CLOCK_DIVIDER);

    // Configure DMA for ADC transfers
    adcDma0 = dma_claim_unused_channel(true);
    adcDma1 = dma_claim_unused_channel(true);
    dmaCfg0 = dma_channel_get_default_config(adcDma0);
    dmaCfg1 = dma_channel_get_default_config(adcDma1);

    channel_config_set_transfer_data_size(&dmaCfg0, DMA_SIZE_16);
    channel_config_set_read_increment(&dmaCfg0, false);
    channel_config_set_write_increment(&dmaCfg0, true);
    channel_config_set_dreq(&dmaCfg0, DREQ_ADC);
    channel_config_set_chain_to(&dmaCfg0, adcDma1);

    channel_config_set_transfer_data_size(&dmaCfg1, DMA_SIZE_16);
    channel_config_set_read_increment(&dmaCfg1, false);
    channel_config_set_write_increment(&dmaCfg1, true);
    channel_config_set_dreq(&dmaCfg1, DREQ_ADC);
    channel_config_set_chain_to(&dmaCfg1, adcDma0);

    dma_set_irq0_channel_mask_enabled((1u<<adcDma0) | (1u<<adcDma1), true);
    irq_set_exclusive_handler(DMA_IRQ_0, adc_interrupt_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    hw_clear_bits(&adc_hw->fcs, ADC_FCS_UNDER_BITS);
    hw_clear_bits(&adc_hw->fcs, ADC_FCS_OVER_BITS);
    adc_fifo_setup(true, true, 1, false, false);
    adc_select_input(0);
    adc_set_round_robin( (1<<AUDIO_I_ADC) | (1<<AUDIO_Q_ADC) );

    dma_channel_configure(adcDma0, &dmaCfg0, adcBuffer0, &adc_hw->fifo, ADC_BUFFER_SIZE, false);
    dma_channel_configure(adcDma1, &dmaCfg1, adcBuffer1, &adc_hw->fifo, ADC_BUFFER_SIZE, false);
    dma_channel_set_irq0_enabled(adcDma0, true);
    dma_channel_set_irq0_enabled(adcDma1, true);
    dma_start_channel_mask(1u << adcDma0);
    adc_run(true);
    
    while (1)
    {
    }
}

// Initialise a PWM for audio output
static int initAudioPWM( uint16_t gpio )
{
    uint16_t audio_slice_num;

    gpio_set_function(gpio, GPIO_FUNC_PWM);
    audio_slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_clkdiv_int_frac(audio_slice_num, pwmDivider, 0);
    pwm_set_wrap(audio_slice_num, AUDIO_WRAP);
    pwm_set_phase_correct(audio_slice_num,false);
    pwm_set_chan_level(audio_slice_num, PWM_CHAN_A, AUDIO_WRAP/2);
    pwm_set_enabled(audio_slice_num, true);

    return audio_slice_num;
}

void ioSetPWMDiv( uint8_t div )
{
    pwmDivider = div;
    pwm_set_clkdiv_int_frac(audio_l_slice_num, div, 0);
    pwm_set_clkdiv_int_frac(audio_r_slice_num, div, 0);
}

uint8_t ioGetPWMDiv( void )
{
    return pwmDivider;
}

void ioSetVolume( uint8_t vol )
{
    if( vol >= MIN_VOLUME && vol <= MAX_VOLUME )
    {
        volume = vol;
    }
}

uint8_t ioGetFilter( void )
{
    return currentFilter;
}

const char *ioGetFilterText( void )
{
    return filters[currentFilter].text;
}

uint8_t ioGetNumFilters( void )
{
    return NUM_FILTERS;
}

void ioSetFilter( uint8_t filter )
{
    if( filter >= 0 && filter < NUM_FILTERS )
    {
        currentFilter = filter;
    }
}

uint8_t ioGetHilbertFilter( void )
{
    return currentHilbertFilter;
}

const char *ioGetHilbertFilterText( void )
{
    return hilbertFilters[currentHilbertFilter].text;
}

uint8_t ioGetNumHilbertFilters( void )
{
    return NUM_HILBERT_FILTERS;
}

void ioSetHilbertFilter( uint8_t filter )
{
    if( filter >= 0 && filter < NUM_HILBERT_FILTERS )
    {
        currentHilbertFilter = filter;
    }
}

u_int8_t ioGetVolume( void )
{
    return volume;
}

#ifdef VARIABLE_SIDETONE_VOLUME
void ioSetSidetoneVolume( uint8_t vol )
{
    if( vol >= MIN_VOLUME && vol <= MAX_VOLUME )
    {
#ifdef PWM_SIDETONE
    pwm_set_chan_level(sidetone_slice_num, PWM_CHAN_A, SIDETONE_WRAP/MAX_SIDETONE_PWM*duty);
#else
        sidetoneVolume = vol;
#endif
    }
}

u_int8_t ioGetSidetoneVolume( void )
{
    return sidetoneVolume;
}
#endif

static struct
{
    uint8_t a;
    uint8_t b;
    uint8_t sw;
}
rotary[NUM_ROTARIES] =
{
    {ROTARY_MAIN_A_GPIO, ROTARY_MAIN_B_GPIO, ROTARY_MAIN_SW_GPIO},
    {ROTARY_VOLUME_A_GPIO, ROTARY_VOLUME_B_GPIO, ROTARY_VOLUME_SW_GPIO},
};

void ioReadRotary( int rotaryNum, bool *pbA, bool *pbB, bool *pbSw )
{
    if( rotaryNum < NUM_ROTARIES)
    {
        *pbA  = !gpio_get(rotary[rotaryNum].a);
        *pbB  = !gpio_get(rotary[rotaryNum].b);
#ifdef ANALOGUE_BUTTONS
        uint16_t result = adcSample[BUTTON_ADC];
        *pbSw = ( (result >= ROTARY_SW_MIN) && (result <= ROTARY_SW_MAX) );
#else
        *pbSw  = !gpio_get(rotary[rotaryNum].sw);
#endif
    }
}

// Read the left and right pushbuttons
#ifdef ANALOGUE_BUTTONS
bool ioReadLeftButton()
{
    uint16_t result = adcSample[BUTTON_ADC];
    return ( (result >= LEFT_BUTTON_MIN) && (result <= LEFT_BUTTON_MAX) );
}

bool ioReadRightButton()
{
    uint16_t result = adcSample[BUTTON_ADC];
    return ( (result >= RIGHT_BUTTON_MIN) && (result <= RIGHT_BUTTON_MAX) );
}
#else

static uint8_t buttonMap[NUM_PUSHBUTTONS] =
{
    PUSHBUTTON_0_GPIO,
    PUSHBUTTON_1_GPIO,
    PUSHBUTTON_2_GPIO,
    PUSHBUTTON_3_GPIO,
};

bool ioReadButton( uint8_t button )
{
    if( button < NUM_PUSHBUTTONS )
    {
        return !gpio_get(buttonMap[button]);
    }
    else
    {
        return false;
    }
}
#endif

// Read the morse dot and dash paddles
bool ioReadDotPaddle()
{
    return !gpio_get(MORSE_PADDLE_DOT_GPIO);
}

bool ioReadDashPaddle()
{
    return !gpio_get(MORSE_PADDLE_DASH_GPIO);
}

// Set the morse output high or low
void ioWriteMorseOutputHigh()
{
    gpio_put(MORSE_OUTPUT_GPIO, 1);
}

void ioWriteMorseOutputLow()
{
    gpio_put(MORSE_OUTPUT_GPIO, 0);
}

// Set the preamp enable on or off
void ioWritePreampOn()
{
    gpio_put(PREAMP_ENABLE_GPIO, 1);
}

void ioWritePreampOff()
{
    gpio_put(PREAMP_ENABLE_GPIO, 0);
}

// Set RX enable high or low
void ioWriteRXEnableHigh()
{
    gpio_put(RX_ENABLE_GPIO, 1);
    bMuteRX = false;
}

void ioWriteRXEnableLow()
{
    bMuteRX = true;
    gpio_put(RX_ENABLE_GPIO, 0);
}

// Switch the sidetone output on or off
void ioWriteSidetoneOn()
{
#ifdef PWM_SIDETONE
    pwm_set_enabled(sidetone_slice_num, true);
#else
    sidetoneOn = true;

    // Always start the sine wave at zero crossing
    currentSinePos = 0;
#endif
}

void ioWriteSidetoneOff()
{
#ifdef PWM_SIDETONE
    pwm_set_enabled(sidetone_slice_num, false);
#else
    sidetoneOn = false;
#endif
}

// Switch the band relay output on or off
void ioWriteBandRelay( uint8_t relay, bool bOn )
{
    if( relay < NUM_RELAYS )
    {
        gpio_put(relayGpio[relay], bOn);
    }
}

// Configure all the I/O we need
void ioInit()
{
    // Force 3V3 regulator into PWM mode to reduce noise
    // We can also disable the regulator by tying 3V3_EN to ground
    // and supplying our own clean 3V3 to 3V3(OUT)
    gpioSetOutput(23);
    gpio_put(23, 1);

    gpioSetOutput(MORSE_OUTPUT_GPIO);
    gpioSetOutput(RX_ENABLE_GPIO);

    gpioSetOutput(PREAMP_ENABLE_GPIO);

    gpioSetInputWithPullup(ROTARY_MAIN_A_GPIO);
    gpioSetInputWithPullup(ROTARY_MAIN_B_GPIO);

    gpioSetInputWithPullup(ROTARY_MAIN_SW_GPIO);

    for( int button = 0 ; button < NUM_PUSHBUTTONS ; button++ )
    {
        gpioSetInputWithPullup(buttonMap[button]);
    }

    gpioSetInputWithPullup(ROTARY_VOLUME_A_GPIO);
    gpioSetInputWithPullup(ROTARY_VOLUME_B_GPIO);
    gpioSetInputWithPullup(ROTARY_VOLUME_SW_GPIO);

    gpioSetInputWithPullup(MORSE_PADDLE_DOT_GPIO);
    gpioSetInputWithPullup(MORSE_PADDLE_DASH_GPIO);

    for( int i = 0 ; i < NUM_RELAYS ; i++ )
    {
        gpioSetOutput(relayGpio[i]);
    }

    // Set up PWM for audio output
    audio_l_slice_num = initAudioPWM( AUDIO_PWM_L_GPIO );
    audio_r_slice_num = initAudioPWM( AUDIO_PWM_R_GPIO );

    // The second core sets up and uses the ADC
    multicore_launch_core1(core1_main);
}
