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
#define DEFAULT_BACKLIGHT_MODE  backlightAuto

// In auto backlight mode how long to delay before turning off the backlight
#define BACKLIGHT_AUTO_DELAY    5000

// As we are controlling the backlight it should start off
#define BACKLIGHT_STARTS_OFF

// How long the volume is displayed for when it changes
#define VOLUME_DISPLAY_DELAY 1000

// Time for debouncing a button (ms)
#define DEBOUNCE_TIME   100

// Time for a button press to be a long press (ms)
#define LONG_PRESS_TIME 250

// Time for debouncing the rotary pushbutton (ms)
#define ROTARY_BUTTON_DEBOUNCE_TIME   100

// Time for the rotary pushbutton to be a long press (ms)
#define ROTARY_LONG_PRESS_TIME 250

// The number of rotary controls
#define NUM_ROTARIES 2

#define MAIN_ROTARY 0
#define VOLUME_ROTARY 1

// Oscillator chip definitions
// I2C address
#define SI5351A_I2C_ADDRESS 0x60

// The si5351a default crystal frequency and load capacitance
#define DEFAULT_XTAL_FREQ	25000000UL
#define SI_XTAL_LOAD_CAP SI_XTAL_LOAD_8PF

// I2C clock speed
#define I2C_CLOCK_RATE 400000

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

// By default we are using CW mode
#define DEFAULT_TRX_MODE   0

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

#define CLOCK_FREQ 125000000

#define SIDETONE_DIVIDE 3
#define SIDETONE_WRAP (CLOCK_FREQ/SIDETONE_DIVIDE/CW_FREQUENCY)

// Minimum and maximum volume levels
#define MIN_VOLUME 0
#define MAX_VOLUME 40
#define DEFAULT_VOLUME 11
#define DEFAULT_SIDETONE_VOLUME 8
#define MIN_SIDETONE_VOLUME MIN_VOLUME
#define MAX_SIDETONE_VOLUME MAX_VOLUME
#define SIDETONE_VOLUME_INC 1

#define VARIABLE_SIDETONE_VOLUME

// Serial port definitions
#define SERIAL_BAUD 57600

//#define CAT_CONTROL

#define MAX_I_GAIN 32767
#define MIN_I_GAIN -32768
#define DEFAULT_I_GAIN 0//MAX_I_GAIN

#define MAX_Q_GAIN 32767
#define MIN_Q_GAIN -32768
#define DEFAULT_Q_GAIN MAX_Q_GAIN

#define MAX_IQ_GAIN 32767
#define MIN_IQ_GAIN -32768
#define DEFAULT_IQ_GAIN 0

// External I2C EEPROM device address
#define EEPROM_I2C_ADDRESS 0x50

// Maximum I2C reads to count when waiting for a device
// e.g. EEPROM to finish writing
#define MAX_I2C_WAIT_COUNT 1000

// The maximum page size to write to the EEPROM over I2C
// This is a physical restriction in the device.
// Any multi-byte write to the device must be within the same page.
#define EEPROM_DEVICE_PAGE_SIZE 0x80

// Bit mask for the size
#define EEPROM_DEVICE_PAGE_MASK (EEPROM_DEVICE_PAGE_SIZE - 1)

// The maximum number of bytes to write to an I2C device in one go
// No point being bigger than the size of an EEPROM page
// but might be different if there are other devices on the bus
#define MAX_I2C_DATA_WRITE EEPROM_DEVICE_PAGE_SIZE

// The size of the actual EEPROM device
// i.e. not the size of the simulated EEPROM
#define EEPROM_DEVICE_SIZE 65536

// Divide the device into pages. Each page contains an initial copy
// of the EEPROM followed by update records so this page needs to be
// much bigger than the EEPROM size.
// The final page in the device is an index to the current page.
// These pages are not related to the device's write pages.
#define EEPROM_PAGE_SIZE   1024

// The size of the simulated EEPROM which must be much smaller than
// EEPROM_PAGE_SIZE and must also be smaller than EEPROM_DEVICE_PAGE_SIZE
#define EEPROM_SIZE 32

// The display we are using
#define OLED_DISPLAY
//#define LCD_DISPLAY

#ifdef OLED_DISPLAY

#define OLED_HEIGHT  64
#define OLED_WIDTH  128

#define FREQUENCY_FONT fontGrotesk16x32
#define HALF_FREQUENCY_FONT fontArialBold
#define SMALL_FONT fontSinclairS

#define WPM_FONT SMALL_FONT
#define VOLUME_FONT SMALL_FONT
#define PREAMP_FONT SMALL_FONT

#define FREQUENCY_DOT_FONT SMALL_FONT
#define VFO_LETTER_FONT SMALL_FONT
#define MENU_FONT SMALL_FONT
#define FILTER_FONT SMALL_FONT
#define MODE_FONT SMALL_FONT

#endif

// Number of buttons (in addition to those on rotary controls)
#define NUM_PUSHBUTTONS 4

#define RIGHT_BUTTON    0
#define LEFT_BUTTON     1
#define BUTTON_A        2
#define BUTTON_B        3

// Definitions of GPIOs in order for easy reference

#define AUDIO_PWM_L_GPIO        0
#define PUSHBUTTON_2_GPIO       1
#define AUDIO_PWM_R_GPIO        2
#define PUSHBUTTON_3_GPIO       3

// I2C SDA                      4
// I2C SCL                      5

#define RELAY_0_OUTPUT_PIN      6
#define RELAY_1_OUTPUT_PIN      7
#define RELAY_2_OUTPUT_PIN      8
#define RELAY_3_OUTPUT_PIN      9
#define RELAY_4_OUTPUT_PIN     10

#define ROTARY_VOLUME_A_GPIO   11
#define ROTARY_VOLUME_B_GPIO   12
#define ROTARY_VOLUME_SW_GPIO  13

#define MORSE_PADDLE_DASH_GPIO 14
#define MORSE_PADDLE_DOT_GPIO  15

#define MORSE_OUTPUT_GPIO      16

#define PUSHBUTTON_0_GPIO      17
#define PUSHBUTTON_1_GPIO      18

#define ROTARY_MAIN_SW_GPIO    19
#define ROTARY_MAIN_A_GPIO     20
#define ROTARY_MAIN_B_GPIO     21

#define RX_ENABLE_GPIO         22

// Pico SMPS mode              23
// Pico VBUS monitor           24
// Pico LED                    25

#define AUDIO_I_GPIO           26
#define AUDIO_Q_GPIO           27

#define PREAMP_ENABLE_GPIO     28

// Defining these will enable debug outputs for
// checking not spending too long processing
// audio
// They are otherwise used for buttons so they will
// not be available if debug pins enabled
//#define ADC_DEBUG1_OUTPUT_GPIO  1
//#define ADC_DEBUG2_OUTPUT_GPIO  3

#define NUM_ADC 2

// ADC clock is 48MHz. 
// Ultimate sample rate is 8K but we are overclocking and
// then decimating.
#define ADC_SAMPLE_RATE 128000
#define ADC_CLOCK 48000000
#define ADC_CLOCK_DIVIDER (ADC_CLOCK/ADC_SAMPLE_RATE/NUM_ADC - 1)

#define AUDIO_I_ADC    0
#define AUDIO_Q_ADC    1

#define AUDIO_DIVIDE 1

#define AUDIO_WRAP 4094
//#define AUDIO_WRAP 254

#define DEFAULT_FILTER 2
#define DEFAULT_HILBERT_FILTER 3

#define INTERMEDIATE_FREQUENCY 12000
#define BFO_SAMPLE_RATE         8000

#define DEFAULT_MAX_MUTE_FACTOR 200
#define MAX_MUTE_FACTOR 10000
#define MIN_MUTE_FACTOR 1
