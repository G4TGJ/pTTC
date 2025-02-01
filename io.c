/*
 * io.c
 *
 * Created: 15/07/2023
 * Author : Richard Tomlinson G4TGJ
 */ 

#include <string.h>
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
#include "i2s.h"
#include "WM8960.h"

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

// The lowest and highest samples before deciding the input is overloaded
#define ADC_OVERLOAD_LOW  -32700
#define ADC_OVERLOAD_HIGH  32700

#define CW_BUFFER_LEN 128

// Set true if overload detected
bool adcOverload;

// Keep track of overload on the output
//  0=no overflow
//  1=too large
// -1=too small
int outOverload;

// Reset the overload flag every second
#define ADC_OVERLOAD_RESET_COUNT 1000
static int adcOverloadResetCount;

int minValueI, maxValueI;
int minValueQ, maxValueQ;

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

#ifdef INTERPOLATE_96K
#define INPUT_INTERPOLATE_FACTOR DECIMATE_FACTOR_96_48
#else
#define INPUT_INTERPOLATE_FACTOR 1
#endif

#define INPUT_BUFFER_LEN (AUDIO_BUFFER_FRAMES * INPUT_INTERPOLATE_FACTOR)

#define BUFFER_LEN 128

static uint16_t ifShiftPos;

static int iOutBuffer[OUTPUT_BUFFER_LEN];
static int qOutBuffer[OUTPUT_BUFFER_LEN];
static int hilbertBuffer[HILBERT_FILTER_BUFFER_LEN];
static int delayBuffer[HILBERT_FILTER_BUFFER_LEN];
static int leftBuffer[LEFT_BUFFER_LEN];
static int rightBuffer[RIGHT_BUFFER_LEN];

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

static uint16_t iInPos9648, iOutPos9648, iInPos4824, iOutPos4824, iInPos2408, iOutPos2408;
static int iBuffer96k[BUFFER_LEN];
static int iBuffer48k[BUFFER_LEN];
static int iBuffer24k[BUFFER_LEN];
static int iBuffer8k[BUFFER_LEN];

static uint16_t qInPos9648, qOutPos9648, qInPos4824, qOutPos4824, qInPos2408, qOutPos2408;
static int qBuffer96k[BUFFER_LEN];
static int qBuffer48k[BUFFER_LEN];
static int qBuffer24k[BUFFER_LEN];
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

static int inline fir( int sample, uint16_t *current, int *buffer, int bufLen, const int *taps, int numTaps )
{
    firIn( sample,  current, buffer, bufLen );
    return firOut( *current, buffer, bufLen, taps, numTaps );
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
// outPos - Pointer to position in the output buffer
// taps - Pointer to FIR filter taps
// numTaps - The number of FIR taps
// precision - Bits of precision of the FIR
//
// count/factor must be an integer
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

#ifdef INTERPOLATE_96K        
// Oscillator for the IF
static struct sNCO ncoIF;
#endif

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

// Process a pair of I and Q samples and create the left and right outputs
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

    // Apply AGC
    out = applyAGC( out );

    // Apply the volume
    out = (out*volumeMultiplier[volume])/VOLUME_PRECISION;

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

    if( actualSidetoneVolume > 0 )
    {
        int sidetone = (sinewave()*volumeMultiplier[actualSidetoneVolume])/VOLUME_PRECISION;
        *outLeft += sidetone;
        *outRight += sidetone;
    }

    if( *outLeft < ADC_OVERLOAD_LOW || *outLeft > ADC_OVERLOAD_HIGH || *outRight < ADC_OVERLOAD_LOW || *outRight > ADC_OVERLOAD_HIGH )
    {
        outOverload = true;
    }
}

// Shift the incoming signal by the IF
static inline void shiftIF()
{
    // Shift the frequency down by the IF
    for( int p = 0 ; p < INPUT_BUFFER_LEN ; p++ )
    {
#ifdef INTERPOLATE_96K        
        complexMixer( &iBuffer96k[ifShiftPos], &qBuffer96k[ifShiftPos], &ncoIF );
#else
        if( ifBelow )
        {
            shiftFrequencyDown( &iBuffer48k[ifShiftPos], &qBuffer48k[ifShiftPos] );
        }
        else
        {
            shiftFrequencyUp( &iBuffer48k[ifShiftPos], &qBuffer48k[ifShiftPos] );
        }
#endif
        ifShiftPos = (ifShiftPos+1) & (BUFFER_LEN-1);
    }
}

// Process the incoming buffers of I and Q samples and create the outgoing
// left and right channel buffers
static inline void processAudio()
{
    int i;

    if( applyRoofingFilter )
    {
        // Shift the incoming signal by the IF
        shiftIF();

#ifdef INTERPOLATE_96K        
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
 
        // Process the decimated I and Q to produce left and right output
        for (i = 0 ; i < AUDIO_BUFFER_FRAMES / DECIMATE_FACTOR_48_24 / DECIMATE_FACTOR_24_8 ; i++)
        {
            processIQ( iBuffer8k[outPos], qBuffer8k[outPos], &leftOutputBuffer[outPos], &rightOutputBuffer[outPos] );
            outPos = (outPos+1) & (BUFFER_LEN-1);
        }
    }
    else
    {
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
        clearAdcDebug2();
        adcOverload = false;
        adcOverloadResetCount = 0;
        outOverload = 0;
    }
}

volatile int changeIQPhasing, iqPhasing;

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
#ifdef INTERPOLATE_96K        
        firIn(ix, &iInPos, iBuffer96k, BUFFER_LEN);
        firIn(0,  &iInPos, iBuffer96k, BUFFER_LEN);
#else
        firIn(ix, &iInPos, iBuffer48k, BUFFER_LEN);
#endif

        int qx = input[i+1] >> (I2S_SHIFT - inputShift);
        qx = removeDC( qx, &qxm1, &qym1 );
#ifdef INTERPOLATE_96K        
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


void ioSetIF( void )
{
#ifdef INTERPOLATE_96K        
    // Set the IF oscillator
    ncoSet( &ncoIF, ifBelow ? INTERMEDIATE_FREQUENCY : -INTERMEDIATE_FREQUENCY, CODEC_SAMPLE_RATE * INPUT_INTERPOLATE_FACTOR);
#endif
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

    // Start I2S
    i2s_program_start_slaved(pio0, &i2sConfig, dma_i2s_in_handler, &i2s);

    while (1)
    {
    }
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
    sidetoneOn = true;

    // Always start the sine wave at zero crossing
    currentSinePos = 0;
}

void ioWriteSidetoneOff()
{
    sidetoneOn = false;
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

    // Initialise the CODEC
    WM8960Init();

    // Enable the headphone output
    WM8960SetHeadphoneVolume();

    // The second core runs the DSP code
    multicore_launch_core1(core1_main);
}
