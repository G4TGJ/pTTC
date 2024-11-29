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

// Reset the overload flag every second
#define ADC_OVERLOAD_RESET_COUNT 8000
static int adcOverloadResetCount;

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

static int OutBuffer[OUTPUT_BUFFER_LEN];
static int hilbertBuffer[HILBERT_FILTER_BUFFER_LEN];
static int delayBuffer[HILBERT_FILTER_BUFFER_LEN];
static int leftBuffer[LEFT_BUFFER_LEN];
static int rightBuffer[RIGHT_BUFFER_LEN];

static uint16_t OutCurrentSample;
static uint16_t hilbertCurrentSample;
static uint16_t delayCurrentSample;
static uint16_t leftCurrentSample;
static uint16_t rightCurrentSample;

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

// The selected intermediate frequency
enum eIF intermediateFrequency = IF_8KHZ;

int iGain = DEFAULT_I_GAIN;
int qGain = DEFAULT_Q_GAIN;
int iqGain = DEFAULT_IQ_GAIN;

bool applyGains;
bool adjustPhase;

volatile uint32_t maxInput;

static uint8_t pwmDivider = AUDIO_DIVIDE;

static uint32_t idleCount;
uint32_t currentCount;

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
    return result >> precision;
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
#if 1
    int i,j;

    // Process each sample
    for( i = 0 ; i < count ; i++ )
    {
        // Integrate
        accumulator[0] += inBuf[*inPos];
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
#else
    // Remember the start position for the comb filter
    int combPos = *inPos;

    // Integrate each input sample
    for( int i = 0 ; i < count ; i++ )
    {
        acc[*inPos] = inBuf[*inPos];

        for( int j = 0 ; j < order ; j++ )
        {
            // Integrator
            acc[*inPos] = acc[*inPos] + acc[(*inPos-1)&(inBufLen-1)];
        }

        // Move the input pointer
        // Which is also the accumulator pointer
        *inPos = (*inPos+1) & (inBufLen-1);
    }

    // Now process the decimated output
    for( int i = 0 ; i < count/factor ; i++ )
    {
        outBuf[*outPos] = acc[combPos];

        for( int j = 0 ; j < order ; j++ )
        {
            // Comb
            outBuf[*outPos] = (outBuf[*outPos] - acc[(combPos-factor)&(inBufLen-1)]) / factor;
        }

        // Move the comb and output pointers
        combPos = (combPos + factor) & (inBufLen-1);
        *outPos = (*outPos + 1) & (outBufLen-1);
    }
#endif
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

// Calculate the output PWM value and apply the volume
static inline int calcPWM( int out, int vol )
{
    // Apply the volume and convert to unsigned
    int outPWM = ((int32_t)out*vol)/VOLUME_PRECISION + ADC_OFFSET;

    // Don't let the PWM output go too low or too high
    if( outPWM < 0 )
    {
        outPWM = 0;
    }
    else if( outPWM > MAX_PWM_OUT )
    {
        outPWM = MAX_PWM_OUT;
    }

    return outPWM;
}

// Shift the frequency by 1/4 of the sample rate
static inline void shiftFrequency1( int *pI, int *pQ )
{
    int origI;
    static int shiftPos;
    switch( shiftPos )
    {
        case 0:
        default:
            // I=I Q=Q
            shiftPos = 1;
            break;

        case 1:
            origI = *pI;
            *pI = -*pQ;
            *pQ = origI;
            shiftPos = 2;
            break;

        case 2:
            *pI = -*pI;
            *pQ = -*pQ;
            shiftPos = 3;
            break;        

        case 3:
            origI = *pI;
            *pI = *pQ;
            *pQ = -origI;
            shiftPos = 0;
            break;
    }
}

static inline void shiftFrequency2( int *pI, int *pQ )
{
    int origI;
    static int shiftPos;
    switch( shiftPos )
    {
        case 0:
        default:
            // I=I Q=Q
            shiftPos = 1;
            break;

        case 1:
            origI = *pI;
            *pI = *pQ;
            *pQ = -origI;
            shiftPos = 2;
            break;

        case 2:
            *pI = -*pI;
            *pQ = -*pQ;
            shiftPos = 3;
            break;        

        case 3:
            origI = *pI;
            *pI = -*pQ;
            *pQ = origI;
            shiftPos = 0;
            break;
    }
}

#if 1
#define shiftFrequencyDown shiftFrequency1
#define shiftFrequencyUp   shiftFrequency2
#else
// Correct
#define shiftFrequencyDown shiftFrequency2
#define shiftFrequencyUp   shiftFrequency1
#endif

// Numerically controlled oscillator
// Only one instance

// Integer value representing 1.0
#define NCO_SCALE   32768

// The number of bits to shift the phase down
#define NCO_BITS    4

// Length of the cosine table
#define NCO_COS_LEN 4096

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
    printf("ncoSet frequency %d sample rate %d\n", frequency, sampleRate);
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

// Initialised the NCO cosine table
static void ncoInit( void )
{
    for( int i = 0 ; i < NCO_COS_LEN ; i++ )
    {
        ncoCos[i] = cos( 2.0 * M_PI * i / NCO_COS_LEN) * NCO_SCALE;
    }

#if 0
    ncoSet( 8000, 32000 );
    printf("ncoFCW=%d\n", ncoFCW);

    for( int i = 0 ; i < 10 ; i++ )
    {
        printf("%d i=%d q=%d\n", i, ncoOscI(), ncoOscQ());
        ncoOscIncrementPhase();
    }

    ncoSet( -8000, 32000 );
    printf("ncoFCW=%d\n", ncoFCW);

    for( int i = 0 ; i < 10 ; i++ )
    {
        printf("%d i=%d q=%d\n", i, ncoOscI(), ncoOscQ());
        ncoOscIncrementPhase();
    }

#endif
}

static inline void complexMixer( int *pI, int *pQ, struct sNCO *ncoIF )
{
    // Get the next quadrature oscillator value
    int iOsc = ncoOscI(ncoIF);
    int qOsc = ncoOscQ(ncoIF);
    ncoOscIncrementPhase(ncoIF);

    // Multiply to get the frequency shift
    int i = ((*pI * iOsc) - (*pQ * qOsc)) / NCO_SCALE;
    int q = ((*pI * qOsc) + (*pQ * iOsc)) / NCO_SCALE;

    // Store the new values
    *pI = i;
    *pQ = q;
}

// Process the input I and Q samples and generate the output sample
// The input and the output are done in the interrupt handler
// but the processing is done in the main loop
static inline void processAudio()
{
    int out, outI, outQ, inI, inQ, outLeft, outRight, i32, q32, i64, q64;
    static int aveI, aveQ;

    // The volume, either normal or sidetone, to apply to the output
    int vol;

    if( applyRoofingFilter )
    {
#if 1

#if 0
        setAdcDebug2();
        decimate( 2, ADC_BUFFER_SIZE, iInputBuffer, DECIMATE_BUFFER_LEN, &iInInterpolate, iDecimate12864Buffer, DECIMATE_BUFFER_LEN, &iOutInterpolate, interpolateFilterTaps, INTERPOLATE_FILTER_TAP_NUM, INTERPOLATE_FILTER_PRECISION );
        decimate( 2, ADC_BUFFER_SIZE, qInputBuffer, DECIMATE_BUFFER_LEN, &qInInterpolate, qDecimate12864Buffer, DECIMATE_BUFFER_LEN, &qOutInterpolate, interpolateFilterTaps, INTERPOLATE_FILTER_TAP_NUM, INTERPOLATE_FILTER_PRECISION );
        clearAdcDebug2();
        decimate( 2, ADC_BUFFER_SIZE/2, iDecimate12864Buffer, DECIMATE_BUFFER_LEN, &iIn12864, iDecimate6432Buffer, DECIMATE_BUFFER_LEN, &iOut12864, decimate12864FilterTaps, DECIMATE_128_64_FILTER_TAP_NUM, DECIMATE_128_64_FILTER_PRECISION );
        decimate( 2, ADC_BUFFER_SIZE/2, qDecimate12864Buffer, DECIMATE_BUFFER_LEN, &qIn12864, qDecimate6432Buffer, DECIMATE_BUFFER_LEN, &qOut12864, decimate12864FilterTaps, DECIMATE_128_64_FILTER_TAP_NUM, DECIMATE_128_64_FILTER_PRECISION );
#endif

        if( intermediateFrequency == IF_8KHZ )
        {
            setAdcDebug2();
            // Shift the frequency down 8kHz
            for( int p = 0 ; p < INPUT_SIZE ; p++ )
            {
                // Get the input buffer positions allowing for buffer wrap
                int iPos = (iIn12864+p)&(DECIMATE_BUFFER_LEN-1);
                int qPos = (qIn12864+p)&(DECIMATE_BUFFER_LEN-1);
                
                complexMixer( &iInputBuffer[iPos], &qInputBuffer[qPos], &ncoIF );

#if 0
                // Get the input from the buffer
                int iIn = iInputBuffer[iPos];
                int qIn = qInputBuffer[qPos];

                // Get the quadrature oscillator
                int iOsc = ncoOscI(&ncoIF);
                int qOsc = ncoOscQ(&ncoIF);
                ncoOscIncrementPhase(&ncoIF);

                // Multiply to get the frequency shift
                iInputBuffer[iPos] = ((iIn * iOsc) - (qIn * qOsc)) / NCO_SCALE;
                qInputBuffer[qPos] = ((iIn * qOsc) + (qIn * iOsc)) / NCO_SCALE;
#endif
            }
            clearAdcDebug2();
        }

#define CIC_ORDER 3
#define CIC_DECIMATION_FACTOR 16

// Number of bits to shift the CIC result right
// 2^CIC_GAIN_BITS is the gain of the filter at DC
// Equal to log2(CIC_DECIMATION_FACTOR^CIC_ORDER) = CIC_ORDER*log2(CIC_DECIMATION_FACTOR)
// Assuming 12 bit samples and 32 bit words maximum is 20 bits before overflow
#define CIC_GAIN_BITS 12

        static int iCombOutput[CIC_ORDER];
        static int qCombOutput[CIC_ORDER];
        static int iCombPreviousInput[CIC_ORDER];
        static int qCombPreviousInput[CIC_ORDER];

        decimateCIC( CIC_DECIMATION_FACTOR, CIC_ORDER, CIC_GAIN_BITS, INPUT_SIZE, iInputBuffer, DECIMATE_BUFFER_LEN, &iIn12864, iAccumulateBuffer, iCombOutput, iCombPreviousInput, iDecimate1608Buffer, DECIMATE_BUFFER_LEN, &iOut12864 );
        decimateCIC( CIC_DECIMATION_FACTOR, CIC_ORDER, CIC_GAIN_BITS, INPUT_SIZE, qInputBuffer, DECIMATE_BUFFER_LEN, &qIn12864, qAccumulateBuffer, qCombOutput, qCombPreviousInput, qDecimate1608Buffer, DECIMATE_BUFFER_LEN, &qOut12864 );

#if 0
        // Filter and decimate from 128ksps to 64ksps
        decimate( 2, INPUT_SIZE, iInputBuffer, DECIMATE_BUFFER_LEN, &iIn12864, iDecimate6432Buffer, DECIMATE_BUFFER_LEN, &iOut12864, decimate12864FilterTaps, DECIMATE_128_64_FILTER_TAP_NUM, DECIMATE_128_64_FILTER_PRECISION );
        decimate( 2, INPUT_SIZE, qInputBuffer, DECIMATE_BUFFER_LEN, &qIn12864, qDecimate6432Buffer, DECIMATE_BUFFER_LEN, &qOut12864, decimate12864FilterTaps, DECIMATE_128_64_FILTER_TAP_NUM, DECIMATE_128_64_FILTER_PRECISION );

        // Filter and decimate from 64ksps to 32ksps
        // Filter depends on the IF. 8kHz is shifted down in frequency after decimation.
        switch ( intermediateFrequency )
        {
            default:
            case IF_0KHZ:
                decimate( 2, INPUT_SIZE/2, iDecimate6432Buffer, DECIMATE_BUFFER_LEN, &iIn6432, iDecimate3216Buffer, DECIMATE_BUFFER_LEN, &iOut6432, decimate64320FilterTaps, DECIMATE_64_32_0_FILTER_TAP_NUM, DECIMATE_64_32_0_FILTER_PRECISION );
                decimate( 2, INPUT_SIZE/2, qDecimate6432Buffer, DECIMATE_BUFFER_LEN, &qIn6432, qDecimate3216Buffer, DECIMATE_BUFFER_LEN, &qOut6432, decimate64320FilterTaps, DECIMATE_64_32_0_FILTER_TAP_NUM, DECIMATE_64_32_0_FILTER_PRECISION );
                break;

            case IF_2KHZ:
                decimate( 2, INPUT_SIZE/2, iDecimate6432Buffer, DECIMATE_BUFFER_LEN, &iIn6432, iDecimate3216Buffer, DECIMATE_BUFFER_LEN, &iOut6432, decimate64322FilterTaps, DECIMATE_64_32_2_FILTER_TAP_NUM, DECIMATE_64_32_2_FILTER_PRECISION );
                decimate( 2, INPUT_SIZE/2, qDecimate6432Buffer, DECIMATE_BUFFER_LEN, &qIn6432, qDecimate3216Buffer, DECIMATE_BUFFER_LEN, &qOut6432, decimate64322FilterTaps, DECIMATE_64_32_2_FILTER_TAP_NUM, DECIMATE_64_32_2_FILTER_PRECISION );
                break;

            case IF_8KHZ:
                decimate( 2, INPUT_SIZE/2, iDecimate6432Buffer, DECIMATE_BUFFER_LEN, &iIn6432, iDecimate3216Buffer, DECIMATE_BUFFER_LEN, &iOut6432, decimate64328FilterTaps, DECIMATE_64_32_8_FILTER_TAP_NUM, DECIMATE_64_32_8_FILTER_PRECISION );
                decimate( 2, INPUT_SIZE/2, qDecimate6432Buffer, DECIMATE_BUFFER_LEN, &qIn6432, qDecimate3216Buffer, DECIMATE_BUFFER_LEN, &qOut6432, decimate64328FilterTaps, DECIMATE_64_32_8_FILTER_TAP_NUM, DECIMATE_64_32_8_FILTER_PRECISION );
#if 0
                for( int p = 0 ; p < INPUT_SIZE/4 ; p++ )
                {
                    shiftFrequencyDown( &iDecimate3216Buffer[(iIn3216+p) % DECIMATE_BUFFER_LEN],  &qDecimate3216Buffer[(qIn3216+p) % DECIMATE_BUFFER_LEN] );
                }
#endif
                break;
        }

        // Filter and decimate from 32ksps to 16ksps
        decimate( 2, INPUT_SIZE/4,  iDecimate3216Buffer, DECIMATE_BUFFER_LEN, &iIn3216, iDecimate1608Buffer, DECIMATE_BUFFER_LEN, &iOut3216, decimate3216FilterTaps, DECIMATE_32_16_FILTER_TAP_NUM, DECIMATE_32_16_FILTER_PRECISION );
        decimate( 2, INPUT_SIZE/4,  qDecimate3216Buffer, DECIMATE_BUFFER_LEN, &qIn3216, qDecimate1608Buffer, DECIMATE_BUFFER_LEN, &qOut3216, decimate3216FilterTaps, DECIMATE_32_16_FILTER_TAP_NUM, DECIMATE_32_16_FILTER_PRECISION );
#endif

        // Filter and decimate from 16ksps to 8ksps
        // This leaves us with a single pair of I and Q
        decimate( 2, INPUT_SIZE/CIC_DECIMATION_FACTOR, iDecimate1608Buffer, DECIMATE_BUFFER_LEN, &iIn1608, &inI, 1, &iOut08, decimate1608FilterTaps, DECIMATE_16_08_FILTER_TAP_NUM, DECIMATE_16_08_FILTER_PRECISION );
        decimate( 2, INPUT_SIZE/CIC_DECIMATION_FACTOR, qDecimate1608Buffer, DECIMATE_BUFFER_LEN, &qIn1608, &inQ, 1, &qOut08, decimate1608FilterTaps, DECIMATE_16_08_FILTER_TAP_NUM, DECIMATE_16_08_FILTER_PRECISION );

        // Apply the BFO to get the CW tone where we want it
        //complexMixer( &inI, &inQ, &ncoBFO );

#else        
        // At the beginning of the interrupt handler the I and Q samples were copied into
        // the FIR filter buffer.
        // We are decimating by 2 so the outputs are every other pair of samples
        // 32 samples of I and Q decimated by 2
        for( int i = 1 ; i <= INPUT_SIZE/2 ; i++ )
        {
            // 64Ksps
            // 1, 2, 3, 4, 5, 6, 7, 8
            // We are decimating by 2 so move past the next 2 I and Q in the output
            iOut12864 = next( iOut12864, DECIMATE_BUFFER_LEN );
            iOut12864 = next( iOut12864, DECIMATE_BUFFER_LEN );
            qOut12864 = next( qOut12864, DECIMATE_BUFFER_LEN );
            qOut12864 = next( qOut12864, DECIMATE_BUFFER_LEN );

            // Take the output of the 128/64 filter and feed into the 64/32 filter
            i64 = firOut( iOut12864, iDecimate12864Buffer, decimate12864FilterTaps, DECIMATE_128_64_FILTER_TAP_NUM, DECIMATE_BUFFER_LEN, DECIMATE_128_64_FILTER_PRECISION );
            q64 = firOut( qOut12864, qDecimate12864Buffer, decimate12864FilterTaps, DECIMATE_128_64_FILTER_TAP_NUM, DECIMATE_BUFFER_LEN, DECIMATE_128_64_FILTER_PRECISION ),

            firIn( i64, &iPos6432, iDecimate6432Buffer, DECIMATE_BUFFER_LEN );
            firIn( q64, &qPos6432, qDecimate6432Buffer, DECIMATE_BUFFER_LEN );

            // 2, 4, 6, 8
            if( (i%2) == 0 )
            {
                // 32Ksps
                switch ( intermediateFrequency )
                {
                    default:
                    case IF_0KHZ:
                        i32 = firOut( iPos6432, iDecimate6432Buffer, decimate64320FilterTaps, DECIMATE_64_32_0_FILTER_TAP_NUM, DECIMATE_BUFFER_LEN, DECIMATE_64_32_0_FILTER_PRECISION );
                        q32 = firOut( qPos6432, qDecimate6432Buffer, decimate64320FilterTaps, DECIMATE_64_32_0_FILTER_TAP_NUM, DECIMATE_BUFFER_LEN, DECIMATE_64_32_0_FILTER_PRECISION );
                        break;

                    case IF_2KHZ:
                        i32 = firOut( iPos6432, iDecimate6432Buffer, decimate64322FilterTaps, DECIMATE_64_32_2_FILTER_TAP_NUM, DECIMATE_BUFFER_LEN, DECIMATE_64_32_2_FILTER_PRECISION );
                        q32 = firOut( qPos6432, qDecimate6432Buffer, decimate64322FilterTaps, DECIMATE_64_32_2_FILTER_TAP_NUM, DECIMATE_BUFFER_LEN, DECIMATE_64_32_2_FILTER_PRECISION );
                        break;

                    case IF_8KHZ:
                        i32 = firOut( iPos6432, iDecimate6432Buffer, decimate64328FilterTaps, DECIMATE_64_32_8_FILTER_TAP_NUM, DECIMATE_BUFFER_LEN, DECIMATE_64_32_8_FILTER_PRECISION );
                        q32 = firOut( qPos6432, qDecimate6432Buffer, decimate64328FilterTaps, DECIMATE_64_32_8_FILTER_TAP_NUM, DECIMATE_BUFFER_LEN, DECIMATE_64_32_8_FILTER_PRECISION );
                        shiftFrequencyDown( &i32, &q32 );
                        break;
                }

                firIn( i32, &iPos3216, iDecimate3216Buffer, DECIMATE_BUFFER_LEN );
                firIn( q32, &qPos3216, qDecimate3216Buffer, DECIMATE_BUFFER_LEN );
            }

            // 4, 8
            if( (i%4) == 0 )
            {
                // 16Ksps
                firIn( firOut( iPos3216, iDecimate3216Buffer, decimate3216FilterTaps, DECIMATE_32_16_FILTER_TAP_NUM, DECIMATE_BUFFER_LEN, DECIMATE_32_16_FILTER_PRECISION ), 
                    &iPos168, iDecimate168Buffer, DECIMATE_BUFFER_LEN );
                firIn( firOut( qPos3216, qDecimate3216Buffer, decimate3216FilterTaps, DECIMATE_32_16_FILTER_TAP_NUM, DECIMATE_BUFFER_LEN, DECIMATE_32_16_FILTER_PRECISION ), 
                    &qPos168, qDecimate168Buffer, DECIMATE_BUFFER_LEN );
            }
        }
        // 8Ksps
        inI = firOut( iPos168, iDecimate168Buffer, decimate168FilterTaps, DECIMATE_16_8_FILTER_TAP_NUM, DECIMATE_BUFFER_LEN, DECIMATE_16_8_FILTER_PRECISION );
        inQ = firOut( qPos168, qDecimate168Buffer, decimate168FilterTaps, DECIMATE_16_8_FILTER_TAP_NUM, DECIMATE_BUFFER_LEN, DECIMATE_16_8_FILTER_PRECISION );
#endif
    }
    else
    {
        inI = iInputBuffer[iInPos];
        inQ = qInputBuffer[qInPos];
    }
    //clearAdcDebug2();
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

    // If it's a 2kHz IF we shift the frequency here
    // because it's 1/4 of the 8kHz sample rate
    if( intermediateFrequency == IF_2KHZ )
    {
        shiftFrequencyDown( &inI, &inQ );
    }

    //shiftFrequencyUp( &inI, &inQ );

    switch( outputSource )
    {
        case sdrIPQ:
        case sdrIMQ:
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

    static bool bWasMuted;

    // Mute the RX if required
#define MIN_MUTE_FACTOR 800
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
            if( bWasMuted && (muteFactor < MIN_MUTE_FACTOR) )
            {
                muteFactor++;
            }
            else
            {
                bWasMuted = false;
                muteFactor = MIN_MUTE_FACTOR;
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

    out = ((int)out * muteFactor) / MIN_MUTE_FACTOR;

    // Apply the CW filter
    if( currentFilter >= 0 && currentFilter < NUM_FILTERS && filters[currentFilter].taps != NULL)
    {
        out = fir(out, &OutCurrentSample, OutBuffer, OUTPUT_BUFFER_LEN, filters[currentFilter].taps, filters[currentFilter].numTaps, filters[currentFilter].precision);;
    }

    if( currentFilter >= 0 && currentFilter < NUM_FILTERS && filters[currentFilter].binaural )
    {
        // Apply LPF to left and HPF to right to get binaural effect
        outLeft  = fir(out, &leftCurrentSample,  leftBuffer,  LEFT_BUFFER_LEN,  leftFilterTaps,  LEFT_FILTER_TAP_NUM,  LEFT_FILTER_PRECISION);
        outRight = fir(out, &rightCurrentSample, rightBuffer, RIGHT_BUFFER_LEN, rightFilterTaps, RIGHT_FILTER_TAP_NUM, RIGHT_FILTER_PRECISION);
    }
    else
    {
        outLeft = outRight = out;
    }

    if( actualSidetoneVolume > 0 )
    {
        outLeft = outRight = sinewave();
        vol = volumeMultiplier[actualSidetoneVolume];
    }
    else
    {
        vol = volumeMultiplier[volume];
    }

    // Convert the samples to unsigned for the PWM and apply the volume
    outPWMLeft = calcPWM( outLeft, vol);
    outPWMRight = calcPWM( outRight, vol);

#define FACTOR 128
    maxInput = (maxInput*(FACTOR-1) + abs(out))/FACTOR;
//    maxInput += abs(out)*1024;

    // Reset the ADC overload flag every second
    adcOverloadResetCount++;
    if( adcOverloadResetCount > ADC_OVERLOAD_RESET_COUNT )
    {
        adcOverload = false;
        adcOverloadResetCount = 0;
    }
}

// Feed from a DMA buffer into the FIR
static inline void feedFir( uint16_t *buf )
{
    int numTaps;
    static int ixm1, iym1;
    static int qxm1, qym1;

    for( int i = 0 ; i < ADC_BUFFER_SIZE/2 ; i++ )
    {
        // Samples alternate between I and Q in DMA buffer and must be converted to signed
        // Remove any DC
        int ix = buf[i*2];
        int iy = ix - ixm1 + (254 * iym1) / 256;
        ixm1 = ix;
        iym1 = iy;
        firIn(iy, &iInPos, iInputBuffer, DECIMATE_BUFFER_LEN);
        firIn(0,  &iInPos, iInputBuffer, DECIMATE_BUFFER_LEN);

        // Remove any DC
        int qx = buf[i*2+1];
        int qy = qx - qxm1 + (254 * qym1) / 256;
        qxm1 = qx;
        qym1 = qy;
        firIn(0,  &qInPos, qInputBuffer, DECIMATE_BUFFER_LEN);
        firIn(qy, &qInPos, qInputBuffer, DECIMATE_BUFFER_LEN);

        if( ix < ADC_OVERLOAD_LOW || ix > ADC_OVERLOAD_HIGH || qx < ADC_OVERLOAD_LOW || qx > ADC_OVERLOAD_HIGH )
        {
            adcOverload = true;
        }
    }
}

static void adc_interrupt_handler()
{
    setAdcDebug1();

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

    // Write the output value determined last time. This adds a slight delay but
    // ensures that the output changes at a consistent time not dependent on the time
    // taken to do all the calculations for the filters.
    pwm_set_chan_level(audio_l_slice_num, PWM_CHAN_A, outPWMLeft);
    pwm_set_chan_level(audio_r_slice_num, PWM_CHAN_A, outPWMRight);
    
    processAudio();

    clearAdcDebug1();
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
    // Because we zero stuff the sample rate
    // is twice the ADC rate
    ncoSet( &ncoIF, INTERMEDIATE_FREQUENCY, 2 * ADC_SAMPLE_RATE );

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
    //sleep_ms(10);
    bMuteRX = false;
}

void ioWriteRXEnableLow()
{
    bMuteRX = true;
    //sleep_ms(2);
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
