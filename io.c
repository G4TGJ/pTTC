/**
 * @file io.c
 * @brief This file contains the implementation of input/output operations.
 * 
 * This file is part of the pTTC project and is located at https://github.com/richard/pTTC/io.c.
 * It includes functions and definitions for handling various I/O operations.
 *
 * Created: 15/07/2023
 * Author : Richard Tomlinson G4TGJ
 */ 

#include <string.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
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

#if defined(I2S_OUTPUT) || defined(I2S_INPUT)
#include "i2s.h"
#endif

#ifdef USE_WM8960
#include "WM8960.h"
#endif

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
    26,
    29,
    32,
    36,
    41,
    46,
    51,
    57,
    64,
    72,
    81,
    91,
    102,
    114,
    128,
    144,
    162,
    181,
    203,
    228,
    256,
    287,
    322,
    362,
    406,
    455,
    511,
    573,
    643,
    722,
    810,
    908,
    1019,
    1144,
    1283,
    1440,
    1615,
    1812,
    2033,
    2282,
    2560,
    2872,
    3223,
    3616,
    4057,
    4552,
    5108,
    5731,
    6430,
    7215,
    8095,
    9083,
    10192,
    11435
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

#ifdef I2S_OUTPUT
static const i2s_config i2sConfig = {CODEC_SAMPLE_RATE,     // Sample rate
                                                   256,     // System clock multiplier - not used
                                                    32,     // Word length
                                                     0,     // System clock GPIO - not used
                                         I2S_DOUT_GPIO,     // DOUT GPIO
                                          I2S_DIN_GPIO,     // DIN GPIO
                                         I2S_BCLK_GPIO,     // Bit clock GPIO (LR clock is on next GPIO)
                                                 false      // false so don't generate system clock
                                    };

static __attribute__((aligned(8))) pio_i2s i2s;

// I2S samples are in the top 16 bits of 32 bit word so need to shift
#define I2S_SHIFT 16

#endif

#ifdef USE_ADC_INPUT

#define ADC_BUFFER_SIZE 32
#define AUDIO_BUFFER_FRAMES ADC_BUFFER_SIZE
#define STEREO_BUFFER_SIZE  (AUDIO_BUFFER_FRAMES * 2)

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

#else

// The lowest and highest samples before deciding the input is overloaded
#define ADC_OVERLOAD_LOW  -32700
#define ADC_OVERLOAD_HIGH  32700

#endif

#define CW_BUFFER_LEN 128

// Set true if overload detected
bool adcOverload;

// Keep track of overload on the output
//  0=no overflow
//  1=too large
// -1=too small
int outOverload;

// Reset the overload flag every second
#define ADC_OVERLOAD_RESET_COUNT (INPUT_SAMPLE_RATE/AUDIO_BUFFER_FRAMES)
static int adcOverloadResetCount;

int minValueI, maxValueI;
int minValueQ, maxValueQ;

#ifdef DISPLAY_MIN_MAX
#define MAX_SAMPLE 32767

// Maximum input and output
int maxIn, maxOut, minIn = MAX_SAMPLE, minOut = MAX_SAMPLE;
int maxIIn, maxQIn, minIIn = MAX_SAMPLE, minQIn = MAX_SAMPLE;
#endif

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

#ifdef USE_ADC_INPUT
// The ADC reads between 0 and 4095 but the DSP needs it to be signed
// so subract this from samples and add back before writing to PWM
#define ADC_OFFSET 2048

// TODO Not for ADC input
#define I2S_SHIFT 16

#endif

#ifdef USE_PWM_OUTPUT
#define MAX_PWM_OUT 4095

// PWM slices for sidetone and audio outputs
static uint16_t sidetone_slice_num;
static uint16_t audio_l_slice_num;
static uint16_t audio_r_slice_num;
#endif

#define BUFFER_LEN 128

static uint16_t ifShiftPos;

// Buffers for storing samples during FIR processing
static int iOutBuffer[OUTPUT_BUFFER_LEN];
static int qOutBuffer[OUTPUT_BUFFER_LEN];
static int hilbertBuffer[HILBERT_FILTER_BUFFER_LEN];
static int delayBuffer[HILBERT_FILTER_BUFFER_LEN];
static int leftBuffer[LEFT_BUFFER_LEN];
static int rightBuffer[RIGHT_BUFFER_LEN];

// Current sample position in the buffers
static uint16_t iOutCurrentSample;
static uint16_t qOutCurrentSample;
static uint16_t hilbertCurrentSample;
static uint16_t delayCurrentSample;
static uint16_t leftCurrentSample;
static uint16_t rightCurrentSample;

// The I and Q channels from the codec may not be exactly in sync
// so add a delay to the I channel
static uint16_t iInPos = DEFAULT_IQ_PHASING;
static uint16_t qInPos;
static uint16_t iIn12864, qIn12864;
static uint16_t iOut12864, qOut12864;

static uint16_t iInInterpolate, iOutInterpolate;
static uint16_t qInInterpolate, qOutInterpolate;

static uint16_t outPos;

static int    iAccumulateBuffer[BUFFER_LEN];
static int    qAccumulateBuffer[BUFFER_LEN];

static uint16_t iInPos25616, iOutPos25616, iInPos9648, iOutPos9648, iInPos4824, iOutPos4824, iInPos2408, iOutPos2408, iInPos1608, iOutPos1608;
static int iBuffer256k[BUFFER_LEN];
static int iBuffer96k[BUFFER_LEN];
static int iBuffer48k[BUFFER_LEN];
static int iBuffer24k[BUFFER_LEN];
static int iBuffer16k[BUFFER_LEN];
static int iBuffer8k[BUFFER_LEN];

static uint16_t qInPos25616, qOutPos25616, qInPos9648, qOutPos9648, qInPos4824, qOutPos4824, qInPos2408, qOutPos2408, qInPos1608, qOutPos1608;
static int qBuffer256k[BUFFER_LEN];
static int qBuffer96k[BUFFER_LEN];
static int qBuffer48k[BUFFER_LEN];
static int qBuffer24k[BUFFER_LEN];
static int qBuffer16k[BUFFER_LEN];
static int qBuffer8k[BUFFER_LEN];

static uint16_t outLRPos;
static int leftOutputBuffer[BUFFER_LEN];
static int rightOutputBuffer[BUFFER_LEN];

static uint16_t leftOutputPos;
static uint16_t rightOutputPos;
static int leftInterpolateBuffer[BUFFER_LEN];
static int rightInterpolateBuffer[BUFFER_LEN];

static int cwBuffer[CW_BUFFER_LEN];
static uint16_t cwCurrentSample;

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

#ifdef USE_PWM_OUTPUT
static uint8_t pwmDivider = AUDIO_DIVIDE;
#endif

static uint32_t idleCount;
uint32_t currentCount;

// Shift the input to provide some initial gain and
// not lose too much precision in filter calculations
int inputShift = DEFAULT_INPUT_SHIFT;

// Main and sidetone volumes (indexes into the multiplier table)
static uint8_t volume = DEFAULT_VOLUME;
static uint8_t sidetoneVolume = DEFAULT_SIDETONE_VOLUME;

/**
 * @brief Delays the input sample by a specified number of samples.
 *
 * This function stores the input sample in a circular buffer and returns
 * the delayed sample from the buffer. The delay is specified in terms of
 * the number of samples.
 *
 * @param sample The input sample to be delayed.
 * @param current Pointer to the current position in the buffer.
 * @param buffer The circular buffer to store the samples.
 * @param bufLen The length of the circular buffer (must be a power of 2).
 * @param delay The number of samples to delay the input sample.
 * @return The delayed sample from the buffer.
 */
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

/**
 * @brief Stores the input sample in a circular buffer.
 *
 * This function stores the input sample in a circular buffer and updates
 * the current position in the buffer. The buffer length must be a power of 2.
 *
 * @param sample The input sample to be stored.
 * @param current Pointer to the current position in the buffer.
 * @param buffer The circular buffer to store the samples.
 * @param bufLen The length of the circular buffer (must be a power of 2).
 */
static inline void firIn( int sample, uint16_t *current, int *buffer, int bufLen )
{
    // Store the new sample
    buffer[*current] = sample;

    // Move to the next sample position, wrapping as needed
    *current = (*current+1) & (bufLen-1);
}

/**
 * @brief Computes the output of a FIR filter.
 *
 * This function calculates the output of a Finite Impulse Response (FIR) filter
 * by convolving the input buffer with the filter taps.
 *
 * @param current The current position in the buffer.
 * @param buffer The circular buffer containing the input samples.
 * @param bufLen The length of the circular buffer (must be a power of 2).
 * @param taps The FIR filter taps.
 * @param numTaps The number of FIR filter taps.
 * @return The filtered sample.
 */
static inline int firOut( uint16_t current, int *buffer, int bufLen, const int *taps, int numTaps )
{
    int tap;
    uint16_t index;
    int result = 0;

    // Calculate the result from the FIR filter
    index = current;
    for( tap = 0 ; tap < numTaps ; tap++ )
    {
        index = (index-1) & (bufLen-1);
        result += ((int) buffer[index]) * taps[tap];
    }

    // Extract the significant bits
    return (result + 16384) / 32768;
}

/**
 * @brief Applies a Finite Impulse Response (FIR) filter to the input sample.
 *
 * This function stores the input sample in a circular buffer and calculates
 * the filtered output by convolving the input buffer with the filter taps.
 *
 * @param sample The input sample to be filtered.
 * @param current Pointer to the current position in the buffer.
 * @param buffer The circular buffer containing the input samples.
 * @param bufLen The length of the circular buffer (must be a power of 2).
 * @param taps The FIR filter taps.
 * @param numTaps The number of FIR filter taps.
 * @return The filtered sample.
 */
static int inline fir( int sample, uint16_t *current, int *buffer, int bufLen, const int *taps, int numTaps )
{
    firIn( sample,  current, buffer, bufLen );
    return firOut( *current, buffer, bufLen, taps, numTaps );
}

/**
 * @brief Decimates the input samples by a specified factor using a FIR filter.
 *
 * This function processes the input samples by applying a FIR filter and then
 * decimating the filtered samples by the specified factor. The decimated samples
 * are stored in the output buffer.
 *
 * @param factor The decimation factor.
 * @param count The number of input samples to process.
 * @param inBuf Pointer to the input buffer containing the samples.
 * @param inBufLen The length of the input buffer (must be a power of 2).
 * @param inPos Pointer to the current position in the input buffer.
 * @param outBuf Pointer to the output buffer to store the decimated samples.
 * @param outBufLen The length of the output buffer (must be a power of 2).
 * @param outPos Pointer to the current position in the output buffer.
 * @param taps Pointer to the FIR filter taps.
 * @param numTaps The number of FIR filter taps.
 */
static inline void decimate( int factor, int count, int *inBuf, int inBufLen, uint16_t *inPos, int *outBuf, int outBufLen, uint16_t *outPos, const int *taps, int numTaps )
{
    // Process each decimation
    for( int i = 0 ; i < count/factor ; i++ )
    {
        int out = firOut( *inPos, inBuf, inBufLen, taps, numTaps );
        firIn( out, outPos, outBuf, outBufLen );

        // Move the input pointer past the decimation factor
        *inPos = (*inPos+factor) & (inBufLen-1);
    }
}

/**
 * @brief Decimates the input samples by a specified factor using a CIC filter.
 *
 * This function processes the input samples by applying a Cascaded Integrator-Comb (CIC) filter 
 * and then decimating the filtered samples by the specified factor. The decimated samples 
 * are stored in the output buffer.
 *
 * @param factor The decimation factor.
 * @param order The order of the CIC filter.
 * @param gainBits The number of bits to shift the gain.
 * @param count The number of input samples to process.
 * @param inBuf Pointer to the input buffer containing the samples.
 * @param inBufLen The length of the input buffer (must be a power of 2).
 * @param inPos Pointer to the current position in the input buffer.
 * @param accumulator Pointer to the accumulator array for the integrator stages.
 * @param combOutput Pointer to the comb output array.
 * @param previousInput Pointer to the previous input array for the comb stages.
 * @param outBuf Pointer to the output buffer to store the decimated samples.
 * @param outBufLen The length of the output buffer (must be a power of 2).
 * @param outPos Pointer to the current position in the output buffer.
 */
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

/**
 * @brief Applies a Hilbert transform to the input sample.
 *
 * This function stores the input sample in a circular buffer and calculates
 * the Hilbert transformed output by convolving the input buffer with the filter taps.
 *
 * @param sample The input sample to be transformed.
 * @param current Pointer to the current position in the buffer.
 * @param buffer The circular buffer containing the input samples.
 * @param bufLen The length of the circular buffer (must be a power of 2).
 * @param taps The Hilbert transform filter taps.
 * @param numTaps The number of Hilbert transform filter taps.
 * @param precision The precision of the filter taps.
 * @return The transformed sample.
 */
static int inline hilbert( int sample, uint16_t *current, int *buffer, int bufLen, const int *taps, int numTaps, int precision )
{
    int tap;
    uint16_t index;
    int result = 0;

    // Store sample in the FIR buffer
    firIn( sample, current, buffer, bufLen );

    // Calculate the result from the FIR filter
    index = *current;
    for( tap = 0 ; tap < numTaps ; tap++ )
    {
        index = (index-1) & (bufLen-1);
        result += ((int) buffer[index]) * taps[tap];
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

/**
 * @brief Applies a CW peaking IIR filter to the input sample.
 *
 * This function processes the input sample using a peaking IIR filter
 * designed for CW signals. It uses a second-order IIR filter with
 * predefined coefficients.
 *
 * @param x The input sample to be filtered.
 * @return The filtered sample.
 */
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

/* Functions used for debugging purposes if the relevant defines are set.
** Used for timing purposes - the output pins can be monitored on a scope
*/

/* State of debug pin 1 */
static bool adcDebugState1;

/**
 * @brief Toggles the state of the ADC debug GPIO pin 1.
 *
 * This function toggles the state of the ADC debug GPIO pin 1, which can be used
 * for debugging purposes to monitor the state of the ADC.
 */
static inline void toggleAdcDebug1()
{
#ifdef ADC_DEBUG1_OUTPUT_GPIO
    adcDebugState1 = !adcDebugState1;
    gpio_put(ADC_DEBUG1_OUTPUT_GPIO, adcDebugState1);
#endif
}

/**
 * @brief Sets the ADC debug state 1 to true and updates the corresponding GPIO pin.
 * 
 * This function sets the adcDebugState1 to true and updates the GPIO pin defined by
 * ADC_DEBUG1_OUTPUT_GPIO to reflect this state. This function is only available if
 * ADC_DEBUG1_OUTPUT_GPIO is defined.
 */
static inline void setAdcDebug1()
{
#ifdef ADC_DEBUG1_OUTPUT_GPIO
    adcDebugState1 = true;
    gpio_put(ADC_DEBUG1_OUTPUT_GPIO, adcDebugState1);
#endif
}

/**
 * @brief Clears the ADC debug state 1 and updates the corresponding GPIO pin.
 * 
 * This function sets the adcDebugState1 to false and updates the GPIO pin defined by
 * ADC_DEBUG1_OUTPUT_GPIO to reflect this state. This function is only available if
 * ADC_DEBUG1_OUTPUT_GPIO is defined.
 */
static inline void clearAdcDebug1()
{
#ifdef ADC_DEBUG1_OUTPUT_GPIO
    adcDebugState1 = false;
    gpio_put(ADC_DEBUG1_OUTPUT_GPIO, adcDebugState1);
#endif
}

/* State of debug pin 2 */
static bool adcDebugState2;

/**
 * @brief Toggles the ADC debug state 2 and updates the corresponding GPIO pin.
 * 
 * This function toggles the adcDebugState2 and updates the GPIO pin defined by
 * ADC_DEBUG2_OUTPUT_GPIO to reflect this state. This function is only available if
 * ADC_DEBUG2_OUTPUT_GPIO is defined.
 */
static inline void toggleAdcDebug2()
{
#ifdef ADC_DEBUG2_OUTPUT_GPIO
    adcDebugState2 = !adcDebugState2;
    gpio_put(ADC_DEBUG2_OUTPUT_GPIO, adcDebugState2);
#endif
}

/**
 * @brief Sets the ADC debug state 2 to true and updates the corresponding GPIO pin.
 * 
 * This function sets the adcDebugState2 to true and updates the GPIO pin defined by
 * ADC_DEBUG2_OUTPUT_GPIO to reflect this state. This function is only available if
 * ADC_DEBUG2_OUTPUT_GPIO is defined.
 */
static inline void setAdcDebug2()
{
#ifdef ADC_DEBUG2_OUTPUT_GPIO
    adcDebugState2 = true;
    gpio_put(ADC_DEBUG2_OUTPUT_GPIO, adcDebugState2);
#endif
}

/**
 * @brief Clears the ADC debug state 2 and updates the corresponding GPIO pin.
 * 
 * This function sets the adcDebugState2 to false and updates the GPIO pin defined by
 * ADC_DEBUG2_OUTPUT_GPIO to reflect this state. This function is only available if
 * ADC_DEBUG2_OUTPUT_GPIO is defined.
 */
static inline void clearAdcDebug2()
{
#ifdef ADC_DEBUG2_OUTPUT_GPIO
    adcDebugState2 = false;
    gpio_put(ADC_DEBUG2_OUTPUT_GPIO, adcDebugState2);
#endif
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

#ifdef USE_PWM_OUTPUT
// Calculate the output PWM value
static inline int calcPWM( int out )
{
    // Convert to unsigned
    int outPWM = out + ADC_OFFSET;

#ifdef DISPLAY_MIN_MAX
    if( outPWM > maxOut )
    {
        maxOut = outPWM;
    }

    if( outPWM < minOut )
    {
        minOut = outPWM;
    }
#endif

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
#endif

/*
* @brief Implementation of a Numerically Controlled Oscillator (NCO) for signal processing.
*
* This is an implementation of a Numerically Controlled Oscillator (NCO) which is used
* to generate precise frequency signals. The NCO is implemented using a cosine lookup table and 
* phase accumulator technique.
*
* The NCO structure and functions provided in this file allow for setting the frequency of the 
* oscillator, generating the next value of the oscillator, and obtaining the I and Q components 
* of the oscillator signal.
*
* The cosine table is initialized using the ncoInit() function, which precomputes the cosine values 
* scaled by a defined factor. The ncoSet() function calculates the frequency control word (FCW) 
* based on the desired frequency and sample rate. The ncoOsc(), ncoOscI(), ncoOscQ(), and 
* ncoOscIncrementPhase() functions are used to generate the oscillator signal and manage the phase 
* of the oscillator.
*
* The NCO structure (sNCO) contains:
* - ncoFCW: The frequency control word, calculated by ncoSet().
* - phase: The current phase of the oscillator.
*
* The cosine table (ncoCos) is statically allocated and populated during initialization.
*
* Usage:
* 1. Initialize the cosine table using ncoInit().
* 2. Set the desired frequency using ncoSet().
* 3. Generate oscillator values using ncoOsc(), ncoOscI(), ncoOscQ(), and ncoOscIncrementPhase().
*
* Example:
* @code
* ncoInit();
* ncoSet(&ncoBFO, 1000, 48000); // Set BFO to 1 kHz with a sample rate of 48 kHz
* int value = ncoOsc(&ncoBFO);  // Get the next oscillator value
* @endcode
*/

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

#if !IF_IS_QUARTER_SAMPLE_RATE        
// Oscillator for the IF
// Only needed if IF is not at quarter sample rate
static struct sNCO ncoIF;
#endif

// Oscillator for the BFO
static struct sNCO ncoBFO;

/**
 * @brief Sets the frequency of the Numerically Controlled Oscillator (NCO).
 *
 * This function initializes the phase of the NCO to zero and calculates the 
 * frequency control word (FCW) based on the desired frequency and sample rate.
 *
 * @param nco Pointer to the NCO structure.
 * @param frequency The desired frequency in Hz.
 * @param sampleRate The sample rate in Hz.
 */
static void ncoSet( struct sNCO *nco, int frequency, int sampleRate)
{
    nco->phase = 0;
    nco->ncoFCW = ((frequency * NCO_COS_LEN) << NCO_BITS) / sampleRate;
}

/**
 * @brief Generates the next value of the Numerically Controlled Oscillator (NCO).
 *
 * This function returns the next value of the NCO by looking up the cosine table
 * based on the current phase. It then increments the phase by the frequency control word (FCW).
 * Use this or use ncoOscI(), ncoOscQ() and ncoOscIncrementPhase()
 *
 * @param nco Pointer to the NCO structure.
 * @return The next value of the NCO.
 */
static inline int ncoOsc( struct sNCO *nco )
{
    int result = ncoCos[nco->phase >> NCO_BITS];
    nco->phase += nco->ncoFCW;
    return result;
}

/**
 * @brief Generates the next I component for the given NCO (Numerically Controlled Oscillator) structure.
 *
 * This function computes the next value of the oscillator based on the state of the provided NCO structure.
 * Must be followed by ncoOscIncrementPhase() to move to the next phase.
 *
 * @param nco Pointer to the sNCO structure that holds the state of the oscillator.
 * @return The next oscillator value as an integer.
 */
static inline int ncoOscI( struct sNCO *nco )
{
    return ncoCos[nco->phase >> NCO_BITS];
}

/**
 * @brief Generates the next Q component for the given NCO (Numerically Controlled Oscillator) structure.
 *
 * This function computes the next value of the oscillator based on the state of the provided NCO structure.
 * Must be followed by ncoOscIncrementPhase() to move to the next phase.
 *
 * @param nco Pointer to the sNCO structure that holds the state of the oscillator.
 * @return The next oscillator value as an integer.
 */
static inline int ncoOscQ( struct sNCO *nco )
{
    return ncoCos[((nco->phase >> NCO_BITS) - NCO_COS_LEN/4)&(NCO_COS_LEN-1)];
}

/**
 * @brief Increments the phase of the Numerically Controlled Oscillator (NCO).
 *
 * This function increments the phase of the NCO by the frequency control word (FCW).
 * It should be called after generating the I and/or Q components of the oscillator signal
 * i.e. after calling ncoOscI() and/or ncoOscQ().
 *
 * @param nco Pointer to the NCO structure.
 */
static inline void ncoOscIncrementPhase( struct sNCO *nco )
{
    nco->phase += nco->ncoFCW;
}

/**
 * @brief Initializes the Numerically Controlled Oscillator (NCO) cosine table.
 *
 * This function precomputes the cosine values scaled by a defined factor and
 * stores them in the ncoCos table. The table is used for generating precise
 * frequency signals in the NCO.
 */
static void ncoInit( void )
{
    for( int i = 0 ; i < NCO_COS_LEN ; i++ )
    {
        ncoCos[i] = cos( 2.0 * M_PI * i / NCO_COS_LEN) * NCO_SCALE;
    }
}

/**
 * @brief Mixes the input I and Q signals with the NCO to shift the frequency.
 *
 * This function performs a complex mixing operation on the input I and Q signals
 * using the Numerically Controlled Oscillator (NCO). It shifts the frequency of
 * the input signals by multiplying them with the I and Q components of the NCO.
 *
 * @param pI Pointer to the input I signal, which will be modified in place.
 * @param pQ Pointer to the input Q signal, which will be modified in place.
 * @param nco Pointer to the NCO structure used for frequency shifting.
 */
static inline void complexMixer( int *pI, int *pQ, struct sNCO *nco )
{
    // Get the next quadrature oscillator value
    int iOsc = ncoOscI(nco);
    int qOsc = ncoOscQ(nco);
    ncoOscIncrementPhase(nco);

    // Multiply to get the frequency shift
    int i = (((*pI * iOsc) - (*pQ * qOsc)) + (NCO_SCALE/2)) / NCO_SCALE;
    int q = (((*pI * qOsc) + (*pQ * iOsc)) + (NCO_SCALE/2)) / NCO_SCALE;

    // Store the new values
    *pI = i;
    *pQ = q;
}

/**
 * @brief Removes the DC component from the input sample.
 *
 * This function applies a high-pass filter to remove the DC component from the input sample.
 * It uses a simple IIR filter to achieve this.
 * Requires two static values to keep track.
 *
 * @param x The input sample from which the DC component is to be removed.
 * @param xm1 Pointer to the previous input sample.
 * @param ym1 Pointer to the previous output sample.
 * @return The input sample with the DC component removed.
 */
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

/**
 * @brief Applies Automatic Gain Control (AGC) to the input sample.
 *
 * This function adjusts the gain of the input sample to maintain a consistent
 * amplitude level. It calculates the amplitude of the input sample, updates
 * the AGC buffer, and adjusts the gain based on the amplitude relative to a
 * predefined threshold.
 *
 * @param in The input sample to which AGC is applied.
 * @return The input sample with AGC applied.
 */
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

/**
 * @brief Applies the volume multiplier to the input sample.
 *
 * This function adjusts the input sample based on the specified volume level.
 * It multiplies the sample by the volume multiplier corresponding to the given
 * volume index and scales it by the volume precision.
 *
 * @param sample The input sample to which the volume is applied.
 * @param volume The volume level index.
 * @return The sample with the volume applied.
 */
static inline int applyVolume( int sample, int volume )
{
    if( volume <= MAX_VOLUME )
    {
        return sample * volumeMultiplier[volume] / VOLUME_PRECISION;
    }
    else
    {
        return sample;
    }
}

/**
 * @brief Enumeration for mute states.
 *
 * This enumeration defines the possible states for muting the audio output.
 * The states include UNMUTED, MUTING, MUTED, and UNMUTING.
 */
enum eMuteState
{
    UNMUTED,
    MUTING,
    MUTED,
    UNMUTING
};

/**
 * @brief Mute states for the main audio and sidetone.
 *
 * These variables represent the current mute state for the main audio output
 * and the sidetone. The main audio output starts unmuted, while the sidetone
 * starts muted.
 */
static volatile enum eMuteState muteState = UNMUTED;
static volatile enum eMuteState sidetoneState = MUTED;

/**
 * @brief Index used for muting the audio output.
 *
 * This variable keeps track of the current mute index, which is used to
 * gradually mute or unmute the audio output to avoid clicks.
 */
static volatile int muteIndex = DEFAULT_VOLUME;

/**
 * @brief Index used for muting the sidetone.
 *
 * This variable keeps track of the current mute index for the sidetone,
 * which is used to gradually mute or unmute the sidetone to avoid clicks.
 */
static volatile int sidetoneIndex;

/**
 * @brief Speed at which the audio is muted or unmuted.
 *
 * This variable defines the speed at which the audio is gradually muted or unmuted
 * to avoid clicks. The value is set to the default mute speed.
 * It works backwards so the higher the value the slower the mute.
 */
int muteSpeed = DEFAULT_MUTE_SPEED;

/**
 * @brief Delay counter for mute operation.
 *
 * This counter is used to control the speed of unmuting
 * the main audio output. It helps in gradually changing
 * the volume to avoid clicks.
 */
static int muteDelayCounter;

/**
 * @brief Mutes the audio output.
 *
 * This function transitions the mute state from UNMUTED to MUTING,
 * allowing the audio output to gradually mute. If the audio is already
 * in the process of muting or unmuting, it will continue to mute.
 *
 * @param pMuteState Pointer to the current mute state.
 * @param pMuteIndex Pointer to the current mute index.
 * @param volume The unmuted volume level.
 */
static inline void mute(volatile enum eMuteState *pMuteState, volatile int *pMuteIndex, int volume)
{
    switch (*pMuteState)
    {
        case UNMUTED:
            *pMuteState = MUTING;

            // Volume index starts at the unmuted volume
            *pMuteIndex = volume;
            break;

        case MUTING:
            break;

        case MUTED:
            break;

        case UNMUTING:
            *pMuteState = MUTING;
            break;
    }
}

/**
 * @brief Unmutes the audio output.
 *
 * This function transitions the mute state from MUTING or MUTED to UNMUTING,
 * allowing the audio output to gradually return to the unmuted state.
 *
 * @param pMuteState Pointer to the current mute state.
 * @param pMuteIndex Pointer to the current mute index.
 * @param volume The unmuted volume level.
 */
static inline void unmute(volatile enum eMuteState *pMuteState, volatile int *pMuteIndex, int volume)
{
    switch (*pMuteState)
    {
        case UNMUTED:
            break;

        case MUTING:
            *pMuteState = UNMUTING;
            break;

        case MUTED:
            *pMuteState = UNMUTING;
            break;

        case UNMUTING:
            break;
    }
}

/**
 * @brief Applies mute to the input sample based on the current mute state.
 *
 * This function adjusts the input sample based on the current mute state.
 * It gradually mutes or unmutes the sample to avoid clicks.
 * Needs to be called on every output sample i.e. 8kHz.
 *
 * @param sample The input sample to be muted or unmuted.
 * @param pMuteState Pointer to the current mute state.
 * @param pMuteIndex Pointer to the current mute index.
 * @param volume The unmuted volume level.
 * @param delay Pointer to the unmute delay counter. If null then no delay is applied.
 * @return The muted or unmuted sample.
 */
static inline int applyMute(int sample, volatile enum eMuteState *pMuteState, volatile int *pMuteIndex, int volume, int *pDelay)
{
    // The actual volume we will apply
    int vol;

    switch (*pMuteState)
    {
        case UNMUTED:
            break;

        case MUTING:
            if (*pMuteIndex > 0)
            {
                // Move down the volume table
                (*pMuteIndex)--;
            }
            else
            {
                // Now fully muted
                *pMuteState = MUTED;
            }
            break;

        case MUTED:
            break;

        case UNMUTING:
            if (*pMuteIndex < volume)
            {
                // Only use the delay if non null
                if( pDelay )
                {
                    // We unmute more slowly to avoid clicks
                    (*pDelay)++;
                    if( *pDelay >= muteSpeed )
                    {
                        *pDelay = 0;

                        // Move up the volume table
                        (*pMuteIndex)++;
                    }
                }
                else
                {
                    // Move up the volume table
                    (*pMuteIndex)++;
                }
            }
            else
            {
                // Now fully unmuted
                *pMuteState = UNMUTED;
            }
            break;
    }

    // If unmuted then the input sample is untouched
    if (*pMuteState == UNMUTED)
    {
        vol = volume;
    }
    else
    {
        // Otherwise apply the volume for the current mute index
        vol = *pMuteIndex;
    }

    return applyVolume(sample, vol);
}

/**
 * @brief Processes a pair of I and Q samples to create the left and right audio outputs.
 *
 * This function takes in-phase (I) and quadrature (Q) samples, applies various signal processing
 * techniques such as filtering, mixing, and gain adjustments, and produces the left and right
 * audio output samples.
 *
 * @param inI The input in-phase (I) sample.
 * @param inQ The input quadrature (Q) sample.
 * @param outLeft Pointer to the output left audio sample.
 * @param outRight Pointer to the output right audio sample.
 */
static inline void processIQ( int inI, int inQ, int *outLeft, int *outRight )
{
    int out, outI, outQ;

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
        outI = fir(inI, &iOutCurrentSample, iOutBuffer, OUTPUT_BUFFER_LEN, filters[currentFilter].taps, filters[currentFilter].numTaps);
        outQ = fir(inQ, &qOutCurrentSample, qOutBuffer, OUTPUT_BUFFER_LEN, filters[currentFilter].taps, filters[currentFilter].numTaps);
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
            // Apply the BFO to get the CW tone where we want it
            complexMixer( &outI, &outQ, &ncoBFO );

            // Only need I now
            out = outI;
            break;
        case sdrI:
            out = outI;
            break;
        case sdrQ:
            out = outQ;
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

    // Apply AGC
    out = applyAGC( out );

    // Apply any mute
    out = applyMute(out, &muteState, &muteIndex, volume, &muteDelayCounter);

    // Select binaural, peaked or normal output
    switch( outputMode )
    {
        case BINAURAL_OUTPUT:
            // Apply LPF to left and HPF to right to get binaural effect
            *outLeft  = fir(out, &leftCurrentSample,  leftBuffer,  LEFT_BUFFER_LEN,  leftFilterTaps,  LEFT_FILTER_TAP_NUM);
            *outRight = fir(out, &rightCurrentSample, rightBuffer, RIGHT_BUFFER_LEN, rightFilterTaps, RIGHT_FILTER_TAP_NUM);
            break;

        case PEAKED_OUTPUT:
            *outLeft = *outRight = cwPeakIIR(out);
            break;

        default:
            *outLeft = *outRight = out;
            break;
    }

    // Apply sidetone if required.
    if( sidetoneState != MUTED )
    {
        // Apply the side wave and apply gradual mute/unmute to sidetone so it fades up or down to avoid clicks
        int sidetone = applyMute(sinewave(), &sidetoneState, &sidetoneIndex, sidetoneVolume, NULL);

        // Apply the sidetone to the left and right channels
        *outLeft += sidetone;
        *outRight += sidetone;
    }

#ifndef USE_PWM_OUTPUT
    // Check for overload
    if( *outLeft < ADC_OVERLOAD_LOW || *outLeft > ADC_OVERLOAD_HIGH || *outRight < ADC_OVERLOAD_LOW || *outRight > ADC_OVERLOAD_HIGH )
    {
        outOverload = true;
    }
#endif
}

/**
 * @brief Shifts the frequency of the input signal by the intermediate frequency (IF).
 *
 * This function shifts the frequency of the input signal by the intermediate frequency (IF)
 * using a complex mixer or frequency shifter. The direction of the shift (up or down) is determined
 * by the ifBelow flag.
 */
static inline void shiftIF()
{
    // Shift the frequency down by the IF
    for( int p = 0 ; p < AUDIO_BUFFER_FRAMES ; p++ )
    {
#if IF_IS_QUARTER_SAMPLE_RATE
        // If the IF is at quarter the sample rate we can do a simple shift
        if( ifBelow )
        {
            shiftFrequencyDown( &iBuffer48k[ifShiftPos], &qBuffer48k[ifShiftPos] );
        }
        else
        {
            shiftFrequencyUp( &iBuffer48k[ifShiftPos], &qBuffer48k[ifShiftPos] );
        }
#else
        // Use the complex mixer to shift by the IF
        complexMixer( &iInputBuffer[ifShiftPos], &qInputBuffer[ifShiftPos], &ncoIF );
#endif
        ifShiftPos = (ifShiftPos+1) & (BUFFER_LEN-1);
    }
}

/**
 * @brief Processes the incoming buffers of I and Q samples and creates the outgoing left and right channel buffers.
 *
 * This function applies various signal processing techniques such as filtering, decimation, and mixing
 * to the input I and Q samples to produce the left and right audio output samples.
 */
static inline void processAudio()
{
    int i;

    if( applyRoofingFilter )
    {
        // Shift the incoming signal by the IF
        shiftIF();

#if INPUT_SAMPLE_RATE == 96000
        // Filter and decimate from 96ksps to 48ksps
        decimate( DECIMATE_FACTOR_96_48, AUDIO_BUFFER_FRAMES * DECIMATE_FACTOR_96_48, 
                iBuffer96k, BUFFER_LEN, &iInPos9648, 
                iBuffer48k, BUFFER_LEN, &iOutPos9648, 
                decimate9648FilterTaps, DECIMATE_96_48_FILTER_TAP_NUM );
        decimate( DECIMATE_FACTOR_96_48, AUDIO_BUFFER_FRAMES * DECIMATE_FACTOR_96_48, 
                qBuffer96k, BUFFER_LEN, &qInPos9648, 
                qBuffer48k, BUFFER_LEN, &qOutPos9648, 
                decimate9648FilterTaps, DECIMATE_96_48_FILTER_TAP_NUM );
#endif

#if (INPUT_SAMPLE_RATE == 96000) || (INPUT_SAMPLE_RATE == 48000)
        // Filter and decimate from 48ksps to 24ksps
        decimate( DECIMATE_FACTOR_48_24, AUDIO_BUFFER_FRAMES, 
                iBuffer48k, BUFFER_LEN, &iInPos4824, 
                iBuffer24k, BUFFER_LEN, &iOutPos4824, 
                decimate4824FilterTaps, DECIMATE_48_24_FILTER_TAP_NUM );
        decimate( DECIMATE_FACTOR_48_24, AUDIO_BUFFER_FRAMES, 
                qBuffer48k, BUFFER_LEN, &qInPos4824, 
                qBuffer24k, BUFFER_LEN, &qOutPos4824, 
                decimate4824FilterTaps, DECIMATE_48_24_FILTER_TAP_NUM );

        // Filter and decimate from 24ksps to 8ksps
        decimate( DECIMATE_FACTOR_24_8, AUDIO_BUFFER_FRAMES / DECIMATE_FACTOR_48_24, 
                iBuffer24k, BUFFER_LEN, &iInPos2408, 
                iBuffer8k,  BUFFER_LEN, &iOutPos2408, 
                decimate2408FilterTaps, DECIMATE_24_08_FILTER_TAP_NUM );
        decimate( DECIMATE_FACTOR_24_8, AUDIO_BUFFER_FRAMES / DECIMATE_FACTOR_48_24, 
                qBuffer24k, BUFFER_LEN, &qInPos2408, 
                qBuffer8k,  BUFFER_LEN, &qOutPos2408, 
                decimate2408FilterTaps, DECIMATE_24_08_FILTER_TAP_NUM );
        const int totalDecimation = DECIMATE_FACTOR_48_24 * DECIMATE_FACTOR_24_8;
#endif

#if INPUT_SAMPLE_RATE == 256000
// Filter and decimate from 256ksps to 16ksps using CIC
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

        decimateCIC( CIC_DECIMATION_FACTOR, CIC_ORDER, CIC_GAIN_BITS, AUDIO_BUFFER_FRAMES, iBuffer256k, BUFFER_LEN, &iIn12864, iAccumulateBuffer, iCombOutput, iCombPreviousInput, iBuffer16k, BUFFER_LEN, &iOutPos25616 );
        decimateCIC( CIC_DECIMATION_FACTOR, CIC_ORDER, CIC_GAIN_BITS, AUDIO_BUFFER_FRAMES, qBuffer256k, BUFFER_LEN, &qIn12864, qAccumulateBuffer, qCombOutput, qCombPreviousInput, qBuffer16k, BUFFER_LEN, &qOutPos25616 );

        // Filter and decimate from 16ksps to 8ksps
        decimate( DECIMATE_FACTOR_16_8, AUDIO_BUFFER_FRAMES / CIC_DECIMATION_FACTOR, 
                iBuffer16k, BUFFER_LEN, &iInPos1608, 
                iBuffer8k,  BUFFER_LEN, &iOutPos1608, 
                decimate1608FilterTaps, DECIMATE_16_08_FILTER_TAP_NUM );
        decimate( DECIMATE_FACTOR_16_8, AUDIO_BUFFER_FRAMES / CIC_DECIMATION_FACTOR, 
                qBuffer16k, BUFFER_LEN, &qInPos1608, 
                qBuffer8k,  BUFFER_LEN, &qOutPos1608, 
                decimate1608FilterTaps, DECIMATE_16_08_FILTER_TAP_NUM );

        const int totalDecimation = CIC_DECIMATION_FACTOR * DECIMATE_FACTOR_16_8;
#endif

        // Process the decimated I and Q to produce left and right output
        for (i = 0 ; i < AUDIO_BUFFER_FRAMES / totalDecimation ; i++)
        {
            processIQ( iBuffer8k[outPos], qBuffer8k[outPos], &leftOutputBuffer[outPos], &rightOutputBuffer[outPos] );
            outPos = (outPos+1) & (BUFFER_LEN-1);
        }
    }
    else
    {
        // TODO Make this work for other input sample rates
        // Assume 96ksps input sample rate
        // Decimate by copying every nth sample but no filtering
        for (i = 0 ; i < AUDIO_BUFFER_FRAMES ; i++)
        {
            if( !(i % (DECIMATE_FACTOR_48_24 * DECIMATE_FACTOR_24_8)) )
            {
                int iSig = iBuffer96k[iInPos9648];
                int qSig = qBuffer96k[qInPos9648];

                switch( outputSource )
                {
                    case sdrIPQ:
                        iSig = qSig = (iSig + qSig);
                        break;
                    case sdrIMQ:
                        iSig = qSig = (iSig - qSig);
                        break;
                    case sdrI:
                        qSig = iSig;
                        break;
                    case sdrQ:
                        iSig = qSig;
                        break;
                    default:
                        break;
                }

                leftOutputBuffer[outPos] = iSig;
                rightOutputBuffer[outPos] = qSig;

                iInPos9648 = (iInPos9648+1) & (BUFFER_LEN-1);
                qInPos9648 = (qInPos9648+1) & (BUFFER_LEN-1);
                outPos = (outPos+1) & (BUFFER_LEN-1);
            }
        }
    }

    // Reset the ADC and output overload flags every second
    adcOverloadResetCount++;
    if( adcOverloadResetCount > ADC_OVERLOAD_RESET_COUNT )
    {
        toggleAdcDebug2();
        adcOverload = false;
        adcOverloadResetCount = 0;
        outOverload = 0;

#ifdef DISPLAY_MIN_MAX
        minIn = minOut = MAX_SAMPLE;
        minIIn = minQIn = MAX_SAMPLE;
        maxIn = maxOut = 0;
        maxIIn = maxQIn = 0;
#endif
    }
}

volatile int changeIQPhasing, iqPhasing;

#ifdef I2S_INPUT
/**
 * @brief Processes the audio input and generates the audio output.
 *
 * This function processes the audio input samples, applies various signal processing
 * techniques such as filtering, mixing, and gain adjustments, and produces the audio
 * output samples.
 *
 * @param input Pointer to the input buffer containing the audio samples.
 * @param output Pointer to the output buffer to store the processed audio samples.
 */
static inline void process_audio(const int* input, int* output)
{
    int i;
    static int ixm1, iym1;
    static int qxm1, qym1;
    int o = 0;
    int32_t outLeft = 0;
    int32_t outRight = 0;

    if( changeIQPhasing )
    {
        iInPos = (iInPos + changeIQPhasing) & (BUFFER_LEN-1);
        changeIQPhasing = 0;
    }
    iqPhasing = (iInPos - qInPos);

    for( i = 0 ; i < STEREO_BUFFER_SIZE ; i += 2 )
    {
        // Remove any DC
        int ix = input[i] >> (I2S_SHIFT - inputShift);
        ix = removeDC( ix, &ixm1, &iym1 );
#if INPUT_INTERPOLATE_FACTOR == 2
        firIn(ix, &iInPos, iBuffer96k, BUFFER_LEN);
        firIn(0,  &iInPos, iBuffer96k, BUFFER_LEN);
#else
        firIn(ix, &iInPos, iBuffer48k, BUFFER_LEN);
#endif

        int qx = input[i+1] >> (I2S_SHIFT - inputShift);
        qx = removeDC( qx, &qxm1, &qym1 );
#if INPUT_INTERPOLATE_FACTOR == 2
        firIn(0,  &qInPos, qBuffer96k, BUFFER_LEN);
        firIn(qx, &qInPos, qBuffer96k, BUFFER_LEN);
#else
        firIn(qx, &qInPos, qBuffer48k, BUFFER_LEN);
#endif

        if( ix < minValueI )
        {
            minValueI = ix;
        }
        if( qx < minValueQ )
        {
            minValueQ = qx;
        }

        if( ix > maxValueI )
        {
            maxValueI = ix;
        }
        if( qx > maxValueQ )
        {
            maxValueQ = qx;
        }

        if( ix < ADC_OVERLOAD_LOW || ix > ADC_OVERLOAD_HIGH || qx < ADC_OVERLOAD_LOW || qx > ADC_OVERLOAD_HIGH )
        {
            adcOverload = true;
            setAdcDebug2();
        }
    }

    // Process the I/Q input buffers into left/right output buffers
    processAudio();

#if 0
    // Output left and right channels
    for (i = 0; i < AUDIO_BUFFER_FRAMES; i++)
    {
        // Read the decimated output - will zero stuff
        // as the output runs at the same rate as the input
        if( !(i % (DECIMATE_FACTOR_48_24 * DECIMATE_FACTOR_24_8)) )
        {
            outLeft = leftOutputBuffer[outLRPos];
            outRight = rightOutputBuffer[outLRPos];
            outLRPos = (outLRPos+1) & (BUFFER_LEN-1);
        }
        else
        {
            outLeft = outRight = 0;
        }

        // Filter the interpolated left and right outputs
        //outLeft  = fir(outLeft,  &leftOutputPos,  leftInterpolateBuffer,  INTERPOLATE_BUFFER_LEN, interpolateFilterTaps, INTERPOLATE_FILTER_TAP_NUM);
        //outRight = fir(outRight, &rightOutputPos, rightInterpolateBuffer, INTERPOLATE_BUFFER_LEN, interpolateFilterTaps, INTERPOLATE_FILTER_TAP_NUM);

        output[o++] = outLeft  << I2S_SHIFT;
        output[o++] = outRight << I2S_SHIFT;
    }
#else
    // Output left and right channels
    for (i = 0; i < AUDIO_BUFFER_FRAMES; i++)
    {
        // Read the decimated output - will repeat samples
        // as the output runs at the same rate as the input
        if( !(i % (DECIMATE_FACTOR_48_24 * DECIMATE_FACTOR_24_8)) )
        {
            outLeft = leftOutputBuffer[outLRPos] << I2S_SHIFT;
            outRight = rightOutputBuffer[outLRPos] << I2S_SHIFT;
            outLRPos = (outLRPos+1) & (BUFFER_LEN-1);
        }
        output[o++] = outLeft;
        output[o++] = outRight;
    }
#endif
}

/**
 * @brief DMA interrupt handler for I2S input.
 *
 * This function is called when the DMA transfer for I2S input is complete.
 * It processes the audio input samples and generates the audio output samples.
 */
static void dma_i2s_in_handler(void)
{
    setAdcDebug1();

    /* We're double buffering using chained TCBs. By checking which buffer the
     * DMA is currently reading from, we can identify which buffer it has just
     * finished reading (the completion of which has triggered this interrupt).
     */
    if (*(int32_t**)dma_hw->ch[i2s.dma_ch_in_ctrl].read_addr == i2s.input_buffer) {
        // It is inputting to the second buffer so we can overwrite the first
        process_audio((int*)i2s.input_buffer, (int*)i2s.output_buffer);
    } else {
        // It is currently inputting the first buffer, so we write to the second
        process_audio((int*)&i2s.input_buffer[STEREO_BUFFER_SIZE], (int*)&i2s.output_buffer[STEREO_BUFFER_SIZE]);
    }
    dma_hw->ints0 = 1u << i2s.dma_ch_in_data;  // clear the IRQ

    clearAdcDebug1();
}
#endif // I2S_INPUT

#ifdef USE_ADC_INPUT
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
        // This interpolates to 256ksps
        int ix = buf[i*2];
        int iy = removeDC( ix, &ixm1, &iym1 );
        firIn(iy<<inputShift, &iInPos, iBuffer256k, BUFFER_LEN);
        firIn(0,              &iInPos, iBuffer256k, BUFFER_LEN);

        int qx = buf[i*2+1];
        int qy = removeDC( qx, &qxm1, &qym1 );
        firIn(0,              &qInPos, qBuffer256k, BUFFER_LEN);
        firIn(qy<<inputShift, &qInPos, qBuffer256k, BUFFER_LEN);

#ifdef DISPLAY_MIN_MAX
        if( ix > maxIn )
        {
            maxIn = ix;
        }

        if( ix < minIn )
        {
            minIn = ix;
        }

        if( ix > maxIIn )
        {
            maxIIn = ix;
        }

        if( ix < minIIn )
        {
            minIIn = ix;
        }

        if( qx > maxQIn )
        {
            maxQIn = qx;
        }

        if( qx < minQIn )
        {
            minQIn = qx;
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

    static int outPWMPos = 0;

    pwm_set_chan_level(audio_l_slice_num, PWM_CHAN_A, calcPWM( leftOutputBuffer[outPWMPos] ));
    pwm_set_chan_level(audio_r_slice_num, PWM_CHAN_A, calcPWM( rightOutputBuffer[outPWMPos] ));
    outPWMPos = (outPWMPos + 1) & (BUFFER_LEN-1);
    
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

#endif // USE_ADC_INPUT

/**
 * @brief Sets the Intermediate Frequency (IF) oscillator.
 *
 * This function initializes the Numerically Controlled Oscillator (NCO) for the Intermediate Frequency (IF)
 * based on whether the IF is below or above the desired frequency.
 * Only needed if IF is not at quarter the sample rate (can do a simple shift in this case).
 */
void ioSetIF( void )
{
#if !IF_IS_QUARTER_SAMPLE_RATE        
    // Set the IF oscillator
    ncoSet( &ncoIF, ifBelow ? INTERMEDIATE_FREQUENCY : -INTERMEDIATE_FREQUENCY, CODEC_SAMPLE_RATE * INPUT_INTERPOLATE_FACTOR);
#endif
}

/**
 * @brief Main function for core 1. Does all the DSP processing.
 *
 * This function initializes the necessary hardware and peripherals, sets up the
 * Numerically Controlled Oscillator (NCO), and starts the ADC or I2S interface. It then
 * enters an infinite loop to keep the core running.
 */
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

#ifdef USE_ADC_INPUT
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
    
#endif // USE_ADC_INPUT

#ifdef I2S_INPUT    
    // Start I2S
    i2s_program_start_slaved(pio0, &i2sConfig, dma_i2s_in_handler, &i2s);
#endif

    while (1)
    {
    }
}

#ifdef USE_PWM_OUTPUT

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
#endif // USE_PWM_OUTPUT

/**
 * @brief Sets the volume level.
 *
 * This function sets the volume level to the specified value if it is within
 * the valid range defined by MIN_VOLUME and MAX_VOLUME.
 *
 * @param vol The desired volume level.
 */
void ioSetVolume( uint8_t vol )
{
    if( vol >= MIN_VOLUME && vol <= MAX_VOLUME )
    {
        volume = vol;
    }
}

/**
 * @brief Gets the current filter index.
 *
 * This function returns the index of the currently selected filter.
 *
 * @return The index of the current filter.
 */
uint8_t ioGetFilter( void )
{
    return currentFilter;
}

/**
 * @brief Gets the text description of the current filter.
 *
 * This function returns a pointer to the text description of the currently selected filter.
 *
 * @return A pointer to the text description of the current filter.
 */
const char *ioGetFilterText( void )
{
    return filters[currentFilter].text;
}

/**
 * @brief Gets the short text description of the current filter.
 *
 * This function returns a pointer to the short text description of the currently selected filter.
 *
 * @return A pointer to the short text description of the current filter.
 */
const char *ioGetFilterShortText( void )
{
    return filters[currentFilter].shortText;
}

/**
 * @brief Retrieves the number of filters.
 *
 * This function returns the number of filters currently configured.
 *
 * @return The number of filters as an 8-bit unsigned integer.
 */
uint8_t ioGetNumFilters( void )
{
    return NUM_FILTERS;
}

/**
 * @brief Sets the current filter.
 *
 * This function sets the current filter to the specified filter index
 * if it is within the valid range defined by 0 and NUM_FILTERS.
 *
 * @param filter The index of the filter to set.
 */
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

/**
 * @brief Gets the current volume level.
 *
 * This function returns the current volume level.
 *
 * @return The current volume level.
 */
uint8_t ioGetVolume( void )
{
    return volume;
}

#ifdef VARIABLE_SIDETONE_VOLUME
/**
 * @brief Sets the sidetone volume level.
 *
 * This function sets the sidetone volume level to the specified value if it is within
 * the valid range defined by MIN_VOLUME and MAX_VOLUME.
 *
 * @param vol The desired sidetone volume level.
 */
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

/**
 * @brief Get the current sidetone volume.
 *
 * This function retrieves the current volume level of the sidetone.
 *
 * @return The current sidetone volume as an 8-bit unsigned integer.
 */
uint8_t ioGetSidetoneVolume( void )
{
    return sidetoneVolume;
}
#endif

/**
 * @brief Structure to hold the GPIO pins for rotary encoders.
 *
 * This structure defines the GPIO pins used for the A and B channels,
 * as well as the switch, for each rotary encoder.
 */
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

/**
 * @brief Reads the state of a rotary encoder.
 *
 * This function reads the state of a specified rotary encoder and updates the provided
 * boolean pointers with the current state of the encoder's A and B channels, as well as
 * the switch state.
 *
 * @param rotaryNum The number of the rotary encoder to read.
 * @param pbA Pointer to a boolean variable to store the state of the A channel.
 * @param pbB Pointer to a boolean variable to store the state of the B channel.
 * @param pbSw Pointer to a boolean variable to store the state of the switch.
 */
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

// Map between button number and the input GPIO
static uint8_t buttonMap[NUM_PUSHBUTTONS] =
{
    PUSHBUTTON_0_GPIO,
    PUSHBUTTON_1_GPIO,
    PUSHBUTTON_2_GPIO,
    PUSHBUTTON_3_GPIO,
};

/**
 * @brief Reads the state of a pushbutton.
 *
 * This function reads the state of a specified pushbutton and returns
 * true if the button is pressed, false otherwise.
 *
 * @param button The number of the pushbutton to read.
 * @return true if the button is pressed, false otherwise.
 */
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

/**
 * @brief Reads the state of the dot paddle.
 *
 * This function reads the state of the dot paddle and returns
 * true if the paddle is pressed, false otherwise.
 *
 * @return true if the dot paddle is pressed, false otherwise.
 */
bool ioReadDotPaddle()
{
    return !gpio_get(MORSE_PADDLE_DOT_GPIO);
}

/**
 * @brief Reads the state of the dash paddle.
 *
 * This function reads the state of the dash paddle and returns
 * true if the paddle is pressed, false otherwise.
 *
 * @return true if the dash paddle is pressed, false otherwise.
 */
bool ioReadDashPaddle()
{
    return !gpio_get(MORSE_PADDLE_DASH_GPIO);
}

/**
 * @brief Sets the Morse output to a high state.
 *
 * This function configures the output pin used for Morse code signaling
 * to a high state, indicating a signal is being transmitted.
 */
void ioWriteMorseOutputHigh()
{
    gpio_put(MORSE_OUTPUT_GPIO, 1);
}

/**
 * @brief Sets the Morse output to a low state.
 *
 * This function configures the output pin used for Morse code signaling
 * to a low state, indicating no signal is being transmitted.
 */
void ioWriteMorseOutputLow()
{
    gpio_put(MORSE_OUTPUT_GPIO, 0);
}

/**
 * @brief Sets the preamp enable to on state.
 *
 * This function configures the preamp enable GPIO pin to a high state,
 * indicating that the preamp is enabled.
 */
void ioWritePreampOn()
{
    gpio_put(PREAMP_ENABLE_GPIO, 1);
}

/**
 * @brief Sets the preamp enable to off state.
 *
 * This function configures the preamp enable GPIO pin to a low state,
 * indicating that the preamp is disabled.
 */
void ioWritePreampOff()
{
    gpio_put(PREAMP_ENABLE_GPIO, 0);
}

/**
 * @brief Sets the RX enable to high state.
 *
 * This function configures the RX enable GPIO pin to a high state,
 * indicating that the receiver is enabled. It also unmutes.
 */
void ioWriteRXEnableHigh()
{
    gpio_put(RX_ENABLE_GPIO, 1);
    unmute( &muteState, &muteIndex, volume );
}

/**
 * @brief Sets the RX enable to low state.
 *
 * This function configures the RX enable GPIO pin to a low state,
 * indicating that the receiver is disabled. It also mutes.
 */
void ioWriteRXEnableLow()
{
    mute( &muteState, &muteIndex, volume );
    gpio_put(RX_ENABLE_GPIO, 0);
}

/**
 * @brief Switches the sidetone output on.
 *
 * This function enables the sidetone output and ensures that the sine wave
 * starts at a zero crossing to avoid clicks.
 */
void ioWriteSidetoneOn()
{
    // Unmute the sidetone gradually to avoid clicks
    unmute( &sidetoneState, &sidetoneIndex, sidetoneVolume );

    // Always start the sine wave at zero crossing
    currentSinePos = 0;
}

/**
 * @brief Switches the sidetone output off.
 *
 * This function disables the sidetone output.
 */
void ioWriteSidetoneOff()
{
    // Mute the sidetone gradually to avoid clicks
    mute( &sidetoneState, &sidetoneIndex, sidetoneVolume );
}

/**
 * @brief Controls the state of a band relay.
 *
 * This function sets the specified band relay to the desired state (on or off).
 *
 * @param relay The index of the relay to control.
 * @param bOn The desired state of the relay (true for on, false for off).
 */
void ioWriteBandRelay( uint8_t relay, bool bOn )
{
    if( relay < NUM_RELAYS )
    {
        gpio_put(relayGpio[relay], bOn);
    }
}

/**
 * @brief Initializes the I/O configuration.
 *
 * This function sets up the necessary GPIO pins for input and output,
 * initializes the CODEC, and launches the second core for DSP processing.
 */
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

#ifdef USE_WM8960
    // Initialise the CODEC
    WM8960Init();

    // Enable the headphone output
    WM8960SetHeadphoneVolume();
#endif

#ifdef USE_PWM_OUTPUT
    // Set up PWM for audio output
    audio_l_slice_num = initAudioPWM( AUDIO_PWM_L_GPIO );
    audio_r_slice_num = initAudioPWM( AUDIO_PWM_R_GPIO );
#endif // USE_PWM_OUTPUT

    // The second core runs the DSP code
    multicore_launch_core1(core1_main);
}
