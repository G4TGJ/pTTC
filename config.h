#include "pico/stdlib.h"

// General definitions
//typedef uint8_t bool;
#define true 1
#define false 0

#define ULONG_MAX 0xFFFFFFFF

// Address of the LCD display
#define LCD_I2C_ADDRESS 0x27

// Dimensions of the LCD screen
#define LCD_WIDTH 16
#define LCD_HEIGHT 2

#define LCD_RS_GPIO     6
#define LCD_ENABLE_GPIO 7 
#define LCD_DATA_GPIO_0 8
#define LCD_DATA_GPIO_1 9
#define LCD_DATA_GPIO_2 10
#define LCD_DATA_GPIO_3 11

// Length of text buffers
// Set to the be the LCD line length with a safety margin
#define TEXT_BUF_LEN (LCD_WIDTH*2)

// Lines on the display
#define FREQ_LINE  0
#define MENU_LINE  1
#define MORSE_LINE 1
#define WPM_LINE   0

// The position on the screen of the morse WPM setting cursor
#define WPM_COL 12

#define ENABLE_DISPLAY_SPLIT_LINE

// The default backlight mode
#ifdef FIVEBAND
#define DEFAULT_BACKLIGHT_MODE  backlightOn
#else
#define DEFAULT_BACKLIGHT_MODE  backlightAuto
#endif

// In auto backlight mode how long to delay before turning off the backlight
#define BACKLIGHT_AUTO_DELAY    5000

// As we are controlling the backlight it should start off
#define BACKLIGHT_STARTS_OFF   

// ADC used for the buttons with input channel,
// pin control register and interrupt vector
#define BUTTON_ADC        0
#define BUTTON_ADC_GPIO   26

// ADC values for the left, right and rotary buttons.
#define ROTARY_SW_MIN 0
#define ROTARY_SW_MAX 100
#define LEFT_BUTTON_MIN 500
#define LEFT_BUTTON_MAX 1500
#define RIGHT_BUTTON_MIN 2000
#define RIGHT_BUTTON_MAX 3000

#define MORSE_PADDLE_DASH_GPIO 14
#define MORSE_PADDLE_DOT_GPIO  15

#define ROTARY_ENCODER_A_GPIO        20
#define ROTARY_ENCODER_B_GPIO        21

#define RX_ENABLE_GPIO 22

#define MORSE_OUTPUT_GPIO 25

// Time for debouncing a button (ms)
#define DEBOUNCE_TIME   100

// Time for a button press to be a long press (ms)
#define LONG_PRESS_TIME 250

// Time for debouncing the rotary pushbutton (ms)
#define ROTARY_BUTTON_DEBOUNCE_TIME   100

// Time for the rotary pushbutton to be a long press (ms)
#define ROTARY_LONG_PRESS_TIME 250

// Oscillator chip definitions
// I2C address
#define SI5351A_I2C_ADDRESS 0x60

// The si5351a default crystal frequency and load capacitance
#define DEFAULT_XTAL_FREQ	25000000UL
#define SI_XTAL_LOAD_CAP SI_XTAL_LOAD_8PF

// I2C clock speed
#define I2C_CLOCK_RATE 100000

// Transmit and receive clocks. Direct conversion receive uses 2 clocks (0 and 1) for quadrature.
// Superhet uses one clock for the BFO. In this case use clocks 0 and 2 so that BFO can be set
// accurately. TX uses the third clock, the one not used for RX.
#define NUM_CLOCKS 3
#define RX_CLOCK_A 0
#define RX_CLOCK_B (BFOFrequency == 0 ? 1 : 2)
#define TX_CLOCK   (BFOFrequency == 0 ? 2 : 1)

// The minimum and maximum crystal frequencies in the setting menu
// Have to allow for adjusting above or below actual valid crystal range
#define MIN_XTAL_FREQUENCY 24000000
#define MAX_XTAL_FREQUENCY 28000000

// Morse definitions
// Frequency of CW tone
#define CW_FREQUENCY 700

// The receive frequency needs to be offset from the dial frequency
// to receive with the correct tone.
#define RX_OFFSET CW_FREQUENCY

// Default, minimum and maximum morse speed in wpm
#define DEFAULT_MORSE_WPM 20
#define MIN_MORSE_WPM 5
#define MAX_MORSE_WPM 40

// Default morse keyer mode
#define DEFAULT_KEYER_MODE 0

// How often to update the display
#define DISPLAY_INTERVAL 50

// Default, min and max BFO frequency
// 0 means direct conversion
#define DEFAULT_BFO_FREQ	0UL
#define MIN_BFO_FREQUENCY	0UL
#define MAX_BFO_FREQUENCY	99999999UL

// Position of default band in the frequency table defined in main.c
// The band is stored in NVRAM so this is only used on first power up
#define DEFAULT_BAND 7

// By default we are not using CW-Reverse mode
#define DEFAULT_CWREVERSE false

#define RELAY_STATE_160M   0
#define TX_ENABLED_160M    false
#define QUICK_VFO_160M     false

#define RELAY_STATE_80M    0
#define TX_ENABLED_80M     false
#define QUICK_VFO_80M      false

#define RELAY_STATE_60M    0
#define TX_ENABLED_60M     false
#define QUICK_VFO_60M      false

#define RELAY_STATE_40M    0
#define TX_ENABLED_40M     true
#define QUICK_VFO_40M      true

#define RELAY_STATE_30M    1
#define TX_ENABLED_30M     true
#define QUICK_VFO_30M      true

#define RELAY_STATE_20M    2
#define TX_ENABLED_20M     true
#define QUICK_VFO_20M      true

#define RELAY_STATE_17M    3
#define TX_ENABLED_17M     true
#define QUICK_VFO_17M      true

#define RELAY_STATE_15M    3
#define TX_ENABLED_15M     true
#define QUICK_VFO_15M      true

#define RELAY_STATE_12M    4
#define TX_ENABLED_12M     true
#define QUICK_VFO_12M      true

#define RELAY_STATE_10M    4
#define TX_ENABLED_10M     true
#define QUICK_VFO_10M      true

// The number of band relays
#define NUM_RELAYS 5

#define RELAY_0_OUTPUT_PIN 6
#define RELAY_1_OUTPUT_PIN 7
#define RELAY_2_OUTPUT_PIN 8
#define RELAY_3_OUTPUT_PIN 9
#define RELAY_4_OUTPUT_PIN 10

#define SIDETONE_GPIO 2

#define CLOCK_FREQ 125000000

#define SIDETONE_DIVIDE 3
#define SIDETONE_WRAP (CLOCK_FREQ/SIDETONE_DIVIDE/CW_FREQUENCY)

#define DEFAULT_SIDETONE_PWM 50
#define MIN_SIDETONE_PWM 2
#define MAX_SIDETONE_PWM 99
#define SIDETONE_PWM_INC 1
#define VARIABLE_SIDETONE_VOLUME

#define EEPROM_SIZE 32

// Serial port definitions
#define SERIAL_BAUD 57600

#define CAT_CONTROL
