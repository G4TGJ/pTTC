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
static const int16_t sine700[NUM_SINE700] =
{
#if 1
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
#else
0,
1070,
1825,
2042,
1657,
784,
-320,
-1330,
-1948,
-1991,
-1448,
-478,
633,
1557,
2023,
1892,
1204,
161,
-930,
-1746,
-2048,
-1746,
-930,
161,
1204,
1892,
2023,
1557,
633,
-478,
-1448,
-1991,
-1948,
-1330,
-320,
784,
1657,
2042,
1825,
1070,
0,
-1070,
-1825,
-2042,
-1657,
-784,
320,
1330,
1948,
1991,
1448,
478,
-633,
-1557,
-2023,
-1892,
-1204,
-161,
930,
1746,
2047,
1746,
930,
-161,
-1204,
-1892,
-2023,
-1557,
-633,
478,
1448,
1991,
1948,
1330,
320,
-784,
-1657,
-2042,
-1825,
-1070,
#endif

#if 0
    128,
    197,
    244,
    255,
    225,
    164,
    92,
    31,
    1,
    11,
    59
#endif
};

static int currentSinePos;

static inline int16_t sinewave()
{
    int16_t val = sine700[currentSinePos];
    currentSinePos = (currentSinePos+1) % NUM_SINE700;

    return val;
}

// Functions to read and write inputs and outputs
// This isolates the main logic from the I/O functions making it
// easy to change the hardware e.g. we have the option to
// connect several inputs to a single ADC pin

static void gpioSetOutput( uint gpio )
{
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
}

static void gpioSetInputWithPullup( uint gpio )
{
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_IN);
    gpio_pull_up(gpio);
}

// The ADC reads between 0 and 4095 but the DSP needs it to be signed
// so subract this from samples and add back before writing to PWM
#define ADC_OFFSET 2048

#if 1
#define PWM_DIVIDER 1
#define MAX_PWM_OUT 4095
#else
// PWM is only 8 bit so divide by this before writing the output
#define PWM_DIVIDER 16
#endif

// PWM slices for sidetone and audio outputs
static uint sidetone_slice_num;
static uint audio_l_slice_num;
static uint audio_r_slice_num;

static volatile uint16_t adcSample[NUM_ADC];

static int16_t IBuffer[ROOFING_FILTER_TAP_NUM];
static int16_t QBuffer[ROOFING_FILTER_TAP_NUM];
static int16_t OutBuffer[FILTER_TAP_NUM];
static int16_t hilbertBuffer[HILBERT_FILTER_TAP_NUM];
static int16_t delayBuffer[HILBERT_FILTER_TAP_NUM];
static int16_t leftBuffer[LEFT_FILTER_TAP_NUM];
static int16_t rightBuffer[RIGHT_FILTER_TAP_NUM];

static int ICurrentSample;
static int QCurrentSample;
static int OutCurrentSample;
static int hilbertCurrentSample;
static int delayCurrentSample;
static int leftCurrentSample;
static int rightCurrentSample;

// The following can be configured in menus so are global
enum sdrSource outputSource;
static int currentFilter = DEFAULT_FILTER;
bool applyRoofingFilter = true;
bool shiftFrequency = true;

int16_t iGain = DEFAULT_I_GAIN;
int16_t qGain = DEFAULT_Q_GAIN;
int16_t iqGain = DEFAULT_IQ_GAIN;

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

static inline int16_t delay( int16_t sample, int *current, int16_t *buffer, int delay )
{
    int i;
    int16_t result = 0;

    // The delayed value is where we are currently pointed to in the delay buffer
    result = buffer[*current];

    // Store the new sample
    buffer[*current] = sample;

    // Move to the next sample position, wrapping as needed
    *current = (*current+1) % delay;

    return result;
}

static inline void firIn( int16_t sample, int *current, int16_t *buffer, int numTaps )
{
    // Store the new sample
    buffer[*current] = sample;

    // Move to the next sample position, wrapping as needed
    (*current)++;
    if( *current >= numTaps )
    {
        *current = 0;
    }
}

static inline int16_t firOut( int *current, int16_t *buffer, const int *taps, int numTaps, int precision )
{

#if 1
    int i;
    int index;
    int32_t result = 0;

    // Calculate the result from the FIR filter
    index = *current;
    for( int i = 0 ; i < numTaps ; i++ )
    {
        index--;
        if( index < 0 )
        {
            index = numTaps-1;
        }
        result += ((int32_t) buffer[index]) * taps[i];
    }
#else
    uint8_t i;
    uint8_t index;
    int32_t result = 0;

    // Calculate the result from the FIR filter
    index = *current;
    i = 0;
    do
    {
        index--;
        result += ((int32_t) buffer[index]) * taps[i];
        i++;
    } while( i != 255);
#endif

    // Extract the significant bits
    return (result >> precision);
}

 static int16_t inline fir( int16_t sample, int *current, int16_t *buffer, const int *taps, int numTaps, int precision )
{
    firIn( sample, current, buffer,       numTaps );
    return firOut( current, buffer, taps, numTaps, precision );
}

#if 0
static int16_t fir( int16_t sample, int *current, int16_t *buffer, const int *taps, int numTaps, int precision )
{
    int i;
    int index;
    int32_t result = 0;

    // Store the new sample
    buffer[*current] = sample;

    // Move to the next sample position, wrapping as needed
    (*current)++;
    if( *current >= numTaps )
    {
        *current = 0;
    }

    // Calculate the result from the FIR filter
    index = *current;

    for( i = 0 ; i < numTaps ; i++ )
    {
        index--;
        if( index < 0 )
        {
            index = numTaps-1;
        }
        result += ((int32_t) buffer[index]) * taps[i];
    }

    // Extract the significant bits
    return (result >> precision);
}
#endif

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
static int16_t outPWMLeft;
static int16_t outPWMRight;

// Calculate the output PWM value and apply the volume
static inline int16_t calcPWM( int16_t out, int16_t vol )
{
    // Apply the volume and convert to unsigned
    int16_t outPWM = (((int32_t)out*vol)/VOLUME_PRECISION + ADC_OFFSET)/PWM_DIVIDER;

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

// Process the input I and Q samples and generate the output sample
// The input and the output are done in the interrupt handler
// but the processing is done in the main loop
static inline void processAudio()
{
    int16_t out, outI, outQ, inI, inQ, outLeft, outRight;
    static int aveI, aveQ;

    // The volume, either normal or sidetone, to apply to the output
    int vol;

    setAdcDebug2();

#if 0
    // Must convert from unsigned sample to a signed sample
    int16_t inI = ((int)adcSample[AUDIO_I_ADC] - ADC_OFFSET)/1;
    int16_t inQ = ((int)adcSample[AUDIO_Q_ADC] - ADC_OFFSET)/1;
#endif

    int iSample = ICurrentSample;
    int qSample = QCurrentSample;

    if( applyRoofingFilter )
    {
        if( shiftFrequency )
        {
            inI = firOut(&ICurrentSample, IBuffer, roofingFilterTaps2, ROOFING_FILTER_TAP_NUM_2, ROOFING_FILTER_PRECISION_2);
            inQ = firOut(&QCurrentSample, QBuffer, roofingFilterTaps2, ROOFING_FILTER_TAP_NUM_2, ROOFING_FILTER_PRECISION_2);
        }
        else
        {
            inI = firOut(&ICurrentSample, IBuffer, roofingFilterTaps, ROOFING_FILTER_TAP_NUM, ROOFING_FILTER_PRECISION);
            inQ = firOut(&QCurrentSample, QBuffer, roofingFilterTaps, ROOFING_FILTER_TAP_NUM, ROOFING_FILTER_PRECISION);
        }
    }
    else
    {
        inI = IBuffer[iSample];
        inQ = QBuffer[qSample];
    }

#if 0
    // Remove the DC offset from the input signals by subracting the moving average
    aveI = (aveI*7999+inI)/8000;
    aveQ = (aveQ*7999+inQ)/8000;
    inI -= aveI;
    inQ -= aveQ;
#endif
#if 0
    if( inI > maxInput )
    {
        maxInput = inI;
    }
    if( inQ > maxInput )
    {
        maxInput = inQ;
    }
#endif
    // Apply I and Q gains
    if( applyGains )
    {
        inI = inI*iGain/32768;
        inQ = inQ*qGain/32768;
    }

#if 0
    // Apply a 4kHz roofing filter to I and Q
    if( applyRoofingFilter )
    {
        inI = fir(inI, &ICurrentSample, IBuffer, roofingFilterTaps, ROOFING_FILTER_TAP_NUM, ROOFING_FILTER_PRECISION);
        inQ = fir(inQ, &QCurrentSample, QBuffer, roofingFilterTaps, ROOFING_FILTER_TAP_NUM, ROOFING_FILTER_PRECISION);
    }
#endif

    // Adjust phase by adding some Q to I
    if( adjustPhase )
    {
        inI += (inQ * iqGain/32768);
    }

    if( shiftFrequency )
    {
        int16_t origI;
        static int shiftPos;
        switch( shiftPos )
        {
            case 0:
            default:
                // I=I Q=Q
                shiftPos = 1;
                break;

            case 1:
                origI = inI;
                inI = -inQ;
                inQ = origI;
                shiftPos = 2;
                break;

            case 2:
                inI = -inI;
                inQ = -inQ;
                shiftPos = 3;
                break;        

            case 3:
                origI = inI;
                inI = inQ;
                inQ = -origI;
                shiftPos = 0;
                break;
        }
    }

    switch( outputSource )
    {
        case sdrIPQ:
        case sdrIMQ:
            // Apply the hilbert filter to I and delay Q to match
            outI = fir(inI, &hilbertCurrentSample, hilbertBuffer, hilbertFilterTaps, HILBERT_FILTER_TAP_NUM, HILBERT_FILTER_PRECISION);
            outQ = delay(inQ, &delayCurrentSample, delayBuffer, (HILBERT_FILTER_TAP_NUM-1)/2);
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
            out = delay(inI, &delayCurrentSample, delayBuffer, (HILBERT_FILTER_TAP_NUM-1)/2);
            break;
        case sdrQDelayed:
            out = delay(inQ, &delayCurrentSample, delayBuffer, (HILBERT_FILTER_TAP_NUM-1)/2);
            break;
        case sdrIHilbert:
            out = fir(inI, &hilbertCurrentSample, hilbertBuffer, hilbertFilterTaps, HILBERT_FILTER_TAP_NUM, HILBERT_FILTER_PRECISION);
            break;
        case sdrQHilbert:
            out = fir(inQ, &hilbertCurrentSample, hilbertBuffer, hilbertFilterTaps, HILBERT_FILTER_TAP_NUM, HILBERT_FILTER_PRECISION);
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

    // Apply the CW filter
    if( currentFilter >= 0 && currentFilter < NUM_FILTERS && filters[currentFilter].taps != NULL)
    {
        out = fir(out, &OutCurrentSample, OutBuffer, filters[currentFilter].taps, filters[currentFilter].numTaps, filters[currentFilter].precision);;
    }

    static uint8_t actualVolume;
    static uint8_t volumeFadeRate = MAX_VOLUME;
    static uint8_t volumeFadeCount;
    static bool bWasMuted;

    // Mute the RX if required
    if( bMuteRX )
    {
        bWasMuted = true;
        out = 0;
        actualVolume = 0;
        if( volume == 0 )
        {
            volumeFadeRate = MAX_VOLUME;
        }
        else
        {
            volumeFadeRate = MAX_VOLUME / volume + 20;
        }
        volumeFadeCount = 0;
    }
    else
    {
        // On unmute bring the volume up slowly to avoid clicks
        if( bWasMuted && (actualVolume < volume) )
        {
            volumeFadeCount++;
            if( (volumeFadeCount % volumeFadeRate) == 0 )
            {
                actualVolume++;
            }
        }
        else
        {
            bWasMuted = false;
            volumeFadeCount = 0;
            actualVolume = volume;
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

    if( actualSidetoneVolume > 0 )
    {
        out = sinewave();
        vol = volumeMultiplier[actualSidetoneVolume];
    }
    else
    {
        vol = volumeMultiplier[actualVolume];
    }

    if( currentFilter >= 0 && currentFilter < NUM_FILTERS && filters[currentFilter].binaural )
    {
        // Apply LPF to left and HPF to right to get binaural effect
        outLeft  = fir(out, &leftCurrentSample,  leftBuffer,  leftFilterTaps,  LEFT_FILTER_TAP_NUM,  LEFT_FILTER_PRECISION);
        outRight = fir(out, &rightCurrentSample, rightBuffer, rightFilterTaps, RIGHT_FILTER_TAP_NUM, RIGHT_FILTER_PRECISION);
    }
    else
    {
        outLeft = outRight = out;
    }

    // Convert the samples to unsigned for the PWM and apply the volume
    outPWMLeft = calcPWM( outLeft, vol);
    outPWMRight = calcPWM( outRight, vol);

#if 0
    uint16_t out16 = abs(out);
    if( out16 > maxInput )
    {
        maxInput = out16;
    }
#else
#define FACTOR 128
    maxInput = (maxInput*(FACTOR-1) + abs(out))/FACTOR;
#endif
//    maxInput += abs(out)*1024;
    clearAdcDebug2();
}

#define ADC_BUFFER_SIZE 16

static int captureDma;
static dma_channel_config captureCfg;

static int adcDma0;
static int adcDma1;
static dma_channel_config dmaCfg0;
static dma_channel_config dmaCfg1;
static uint16_t adcBuffer0[ADC_BUFFER_SIZE];
static uint16_t adcBuffer1[ADC_BUFFER_SIZE];

// Feed from a DMA buffer into the FIR
static inline void feedFir( uint16_t *buf )
{
    int i;
    for( i = 0 ; i < ADC_BUFFER_SIZE/2 ; i++ )
    {
        // Samples alternate between I and Q in DMA buffer and must be converted to signed
        firIn(((int)buf[i*2]   - ADC_OFFSET), &ICurrentSample, IBuffer, ROOFING_FILTER_TAP_NUM);
        firIn(((int)buf[i*2]+1 - ADC_OFFSET), &QCurrentSample, QBuffer, ROOFING_FILTER_TAP_NUM);
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
static int initAudioPWM( uint gpio )
{
    uint audio_slice_num;

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
