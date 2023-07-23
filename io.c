/*
 * io.c
 *
 * Created: 15/07/2023
 * Author : Richard Tomlinson G4TGJ
 */ 

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "config.h"
#include "io.h"

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

static uint slice_num;

// Configure all the I/O we need
void ioInit()
{
    gpioSetOutput(MORSE_OUTPUT_GPIO);
    gpioSetOutput(RX_ENABLE_GPIO);

    gpioSetInputWithPullup(ROTARY_ENCODER_A_GPIO);
    gpioSetInputWithPullup(ROTARY_ENCODER_B_GPIO);

    gpioSetInputWithPullup(MORSE_PADDLE_DOT_GPIO);
    gpioSetInputWithPullup(MORSE_PADDLE_DASH_GPIO);

    for( int i = 0 ; i < NUM_RELAYS ; i++ )
    {
        gpioSetOutput(relayGpio[i]);
    }

    adc_init();

    adc_gpio_init(BUTTON_ADC_GPIO);
    gpio_pull_up(BUTTON_ADC_GPIO);
    adc_select_input(BUTTON_ADC);

    gpio_set_function(SIDETONE_GPIO, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(SIDETONE_GPIO);
    pwm_set_clkdiv_int_frac(slice_num, SIDETONE_DIVIDE, 0);
    pwm_set_wrap(slice_num, SIDETONE_WRAP);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, SIDETONE_WRAP/MAX_SIDETONE_PWM*DEFAULT_SIDETONE_PWM);
}

#ifdef VARIABLE_SIDETONE_VOLUME
void ioWriteSidetoneDutyCycle( uint8_t duty )
{
    pwm_set_chan_level(slice_num, PWM_CHAN_A, SIDETONE_WRAP/MAX_SIDETONE_PWM*duty);
}
#endif

void ioReadRotary( bool *pbA, bool *pbB, bool *pbSw )
{
    uint16_t result = adc_read();
    *pbA  = !gpio_get(ROTARY_ENCODER_A_GPIO);
    *pbB  = !gpio_get(ROTARY_ENCODER_B_GPIO);
    *pbSw = ( (result >= ROTARY_SW_MIN) && (result <= ROTARY_SW_MAX) );
}

// Read the left and right pushbuttons
bool ioReadLeftButton()
{
    uint16_t result = adc_read();
    return ( (result >= LEFT_BUTTON_MIN) && (result <= LEFT_BUTTON_MAX) );
}

bool ioReadRightButton()
{
    uint16_t result = adc_read();
    return ( (result >= RIGHT_BUTTON_MIN) && (result <= RIGHT_BUTTON_MAX) );
}

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

// Set RX enable high or low
void ioWriteRXEnableHigh()
{
    gpio_put(RX_ENABLE_GPIO, 1);
}

void ioWriteRXEnableLow()
{
    gpio_put(RX_ENABLE_GPIO, 0);
}

// Switch the sidetone output on or off
void ioWriteSidetoneOn()
{
    pwm_set_enabled(slice_num, true);
}

void ioWriteSidetoneOff()
{
    pwm_set_enabled(slice_num, false);
}

// Switch the band relay output on or off
void ioWriteBandRelay( uint8_t relay, bool bOn )
{
    if( relay < NUM_RELAYS )
    {
        gpio_put(relayGpio[relay], bOn);
    }
}

