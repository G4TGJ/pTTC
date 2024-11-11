/*
FIR filters designed with
http://t-filter.engineerjs.com/
*/

#if 0
/*
sampling frequency: 128000 Hz

fixed point precision: 16 bits

* 0 Hz - 11000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 32000 Hz - 64000 Hz
  gain = 0
  desired attenuation = -80 dB
  actual attenuation = n/a

*/

#define DECIMATE_128_64_FILTER_TAP_NUM 25
#define DECIMATE_128_64_FILTER_PRECISION 15

static int decimate12864FilterTaps[DECIMATE_128_64_FILTER_TAP_NUM] =
{
  -14,
  -38,
  -10,
  145,
  347,
  271,
  -372,
  -1272,
  -1328,
  614,
  4487,
  8493,
  10204,
  8493,
  4487,
  614,
  -1328,
  -1272,
  -372,
  271,
  347,
  145,
  -10,
  -38,
  -14
};
#endif

#if 0
/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 128000 Hz

fixed point precision: 16 bits

* 0 Hz - 11000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 32000 Hz - 64000 Hz
  gain = 0
  desired attenuation = -50 dB
  actual attenuation = n/a

*/

#define DECIMATE_128_64_FILTER_TAP_NUM 17
#define DECIMATE_128_64_FILTER_PRECISION 15

static int decimate12864FilterTaps[DECIMATE_128_64_FILTER_TAP_NUM] =
{
  181,
  322,
  -23,
  -970,
  -1509,
  23,
  4112,
  8803,
  10893,
  8803,
  4112,
  23,
  -1509,
  -970,
  -23,
  322,
  181
};
#endif

/*
sampling frequency: 128000 Hz

fixed point precision: 16 bits

* 0 Hz - 11000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 53000 Hz - 64000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define DECIMATE_128_64_FILTER_TAP_NUM 7
#define DECIMATE_128_64_FILTER_PRECISION 15

static int decimate12864FilterTaps[DECIMATE_128_64_FILTER_TAP_NUM] =
{
  -1205,
  -142,
  9414,
  16565,
  9414,
  -142,
  -1205
};

#if 0
/*
sampling frequency: 64000 Hz

fixed point precision: 16 bits

* 0 Hz - 3000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 29000 Hz - 32000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

*/

#define DECIMATE_64_32_0_FILTER_TAP_NUM 9
#define DECIMATE_64_32_0_FILTER_PRECISION 15

static int decimate64320FilterTaps[DECIMATE_64_32_0_FILTER_TAP_NUM] =
{
  -259,
  -1093,
  950,
  9282,
  14999,
  9282,
  950,
  -1093,
  -259
};
#endif

/*
sampling frequency: 64000 Hz

fixed point precision: 16 bits

* 0 Hz - 3000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 16000 Hz - 32000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

*/

#define DECIMATE_64_32_0_FILTER_TAP_NUM 15
#define DECIMATE_64_32_0_FILTER_PRECISION 15

static int decimate64320FilterTaps[DECIMATE_64_32_0_FILTER_TAP_NUM] =
{
  -97,
  -419,
  -753,
  -422,
  1353,
  4499,
  7674,
  9021,
  7674,
  4499,
  1353,
  -422,
  -753,
  -419,
  -97
};
/*
sampling frequency: 64000 Hz

fixed point precision: 16 bits

* 0 Hz - 5000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 27000 Hz - 32000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

*/

#define DECIMATE_64_32_2_FILTER_TAP_NUM 9
#define DECIMATE_64_32_2_FILTER_PRECISION 15

static int decimate64322FilterTaps[DECIMATE_64_32_2_FILTER_TAP_NUM] =
{
  -304,
  -1230,
  945,
  9398,
  15072,
  9398,
  945,
  -1230,
  -304
};

#if 0
/*
sampling frequency: 64000 Hz

fixed point precision: 16 bits

* 0 Hz - 4000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

* 8000 Hz - 11000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 16000 Hz - 32000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

*/

#define DECIMATE_64_32_8_FILTER_TAP_NUM 47
#define DECIMATE_64_32_8_FILTER_PRECISION 15

static int decimate64328FilterTaps[DECIMATE_64_32_8_FILTER_TAP_NUM] =
{
  -59,
  -100,
  -35,
  115,
  175,
  84,
  37,
  151,
  141,
  -301,
  -810,
  -633,
  208,
  684,
  288,
  93,
  1153,
  2175,
  547,
  -3566,
  -5862,
  -2547,
  4207,
  7691,
  4207,
  -2547,
  -5862,
  -3566,
  547,
  2175,
  1153,
  93,
  288,
  684,
  208,
  -633,
  -810,
  -301,
  141,
  151,
  37,
  84,
  175,
  115,
  -35,
  -100,
  -59
};

#else

/*
sampling frequency: 64000 Hz

fixed point precision: 16 bits

* 0 Hz - 11000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 21000 Hz - 32000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

*/

#define DECIMATE_64_32_8_FILTER_TAP_NUM 21
#define DECIMATE_64_32_8_FILTER_PRECISION 15

static int decimate64328FilterTaps[DECIMATE_64_32_8_FILTER_TAP_NUM] =
{
  58,
  146,
  -129,
  -476,
  228,
  1208,
  -332,
  -2876,
  409,
  10216,
  15945,
  10216,
  409,
  -2876,
  -332,
  1208,
  228,
  -476,
  -129,
  146,
  58
};
#endif

#if 1
/*
sampling frequency: 32000 Hz

fixed point precision: 16 bits

* 0 Hz - 4000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 12000 Hz - 16000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define DECIMATE_32_16_FILTER_TAP_NUM 11
#define DECIMATE_32_16_FILTER_PRECISION 15

static int decimate3216FilterTaps[DECIMATE_32_16_FILTER_TAP_NUM] =
{
  373,
  98,
  -2049,
  -171,
  9852,
  16612,
  9852,
  -171,
  -2049,
  98,
  373
};
#endif

#if 0
/*
sampling frequency: 32000 Hz

fixed point precision: 16 bits

* 0 Hz - 3000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 8000 Hz - 16000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

*/

#define DECIMATE_32_16_FILTER_TAP_NUM 21
#define DECIMATE_32_16_FILTER_PRECISION 15

static int decimate3216FilterTaps[DECIMATE_32_16_FILTER_TAP_NUM] =
{
  -36,
  24,
  247,
  397,
  -25,
  -1053,
  -1587,
  20,
  4161,
  8838,
  10907,
  8838,
  4161,
  20,
  -1587,
  -1053,
  -25,
  397,
  247,
  24,
  -36
};
#endif

/*
sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 3000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 4000 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define DECIMATE_16_8_FILTER_TAP_NUM 39
#define DECIMATE_16_8_FILTER_PRECISION 15

static int decimate168FilterTaps[DECIMATE_16_8_FILTER_TAP_NUM] =
{
  124,
  -4,
  -184,
  -25,
  222,
  188,
  -298,
  -370,
  243,
  675,
  -84,
  -995,
  -341,
  1356,
  1130,
  -1647,
  -2856,
  1865,
  10207,
  14456,
  10207,
  1865,
  -2856,
  -1647,
  1130,
  1356,
  -341,
  -995,
  -84,
  675,
  243,
  -370,
  -298,
  188,
  222,
  -25,
  -184,
  -4,
  124
};

// Size of all the decimate buffers
// Must be bigger than all the filters
// plus extra as we feed in more than one
// sample each time
// The worst case is the first decimate filter
// where we feed, say, 16 samples in after each
// ADC DMA so there needs to be enough space to
// run all the taps over the first sample.
#define DECIMATE_BUFFER_LEN 128


/*
sampling frequency: 8000 Hz

fixed point precision: 16 bits

* 0 Hz - 700 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 800 Hz - 4000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define LEFT_FILTER_TAP_NUM 63
#define LEFT_FILTER_PRECISION 15

static int leftFilterTaps[LEFT_FILTER_TAP_NUM] =
{
  1368,
  -1503,
  -860,
  -433,
  -113,
  139,
  315,
  390,
  335,
  166,
  -84,
  -333,
  -497,
  -510,
  -343,
  -29,
  342,
  644,
  761,
  618,
  215,
  -355,
  -924,
  -1284,
  -1246,
  -694,
  367,
  1809,
  3395,
  4829,
  5826,
  6183,
  5826,
  4829,
  3395,
  1809,
  367,
  -694,
  -1246,
  -1284,
  -924,
  -355,
  215,
  618,
  761,
  644,
  342,
  -29,
  -343,
  -510,
  -497,
  -333,
  -84,
  166,
  335,
  390,
  315,
  139,
  -113,
  -433,
  -860,
  -1503,
  1368
};

/*
sampling frequency: 8000 Hz

fixed point precision: 16 bits

* 0 Hz - 600 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

* 700 Hz - 4000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

*/

#define RIGHT_FILTER_TAP_NUM 63
#define RIGHT_FILTER_PRECISION 15

static int rightFilterTaps[RIGHT_FILTER_TAP_NUM] =
{
  194,
  -1126,
  -625,
  -554,
  -476,
  -327,
  -120,
  106,
  304,
  426,
  438,
  330,
  120,
  -148,
  -409,
  -593,
  -641,
  -520,
  -237,
  164,
  596,
  954,
  1124,
  1016,
  583,
  -172,
  -1183,
  -2331,
  -3465,
  -4422,
  -5062,
  27482,
  -5062,
  -4422,
  -3465,
  -2331,
  -1183,
  -172,
  583,
  1016,
  1124,
  954,
  596,
  164,
  -237,
  -520,
  -641,
  -593,
  -409,
  -148,
  120,
  330,
  438,
  426,
  304,
  106,
  -120,
  -327,
  -476,
  -554,
  -625,
  -1126,
  194
};

#if 0
/*
sampling frequency: 64000 Hz

fixed point precision: 16 bits

* 0 Hz - 2000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 2700 Hz - 32000 Hz
  gain = 0
  desired attenuation = -80 dB
  actual attenuation = n/a

*/

#define ROOFING_FILTER_TAP_NUM 321
#define ROOFING_FILTER_PRECISION 15

static int roofingFilterTaps[ROOFING_FILTER_TAP_NUM] =
{
  2,
  2,
  3,
  3,
  4,
  5,
  6,
  6,
  7,
  8,
  8,
  8,
  8,
  7,
  6,
  5,
  4,
  2,
  0,
  -2,
  -4,
  -6,
  -8,
  -10,
  -12,
  -13,
  -14,
  -14,
  -13,
  -12,
  -10,
  -7,
  -4,
  -1,
  3,
  6,
  10,
  13,
  16,
  19,
  20,
  20,
  20,
  18,
  15,
  11,
  7,
  1,
  -5,
  -11,
  -17,
  -23,
  -28,
  -31,
  -34,
  -35,
  -34,
  -32,
  -28,
  -22,
  -15,
  -6,
  3,
  13,
  22,
  31,
  39,
  45,
  50,
  52,
  51,
  48,
  42,
  34,
  23,
  10,
  -4,
  -19,
  -33,
  -47,
  -59,
  -69,
  -76,
  -79,
  -79,
  -74,
  -66,
  -53,
  -37,
  -19,
  2,
  24,
  45,
  66,
  84,
  99,
  109,
  115,
  115,
  108,
  96,
  78,
  55,
  28,
  -3,
  -35,
  -67,
  -97,
  -125,
  -147,
  -164,
  -172,
  -173,
  -164,
  -146,
  -120,
  -86,
  -45,
  1,
  49,
  99,
  146,
  189,
  225,
  251,
  267,
  269,
  258,
  232,
  192,
  138,
  73,
  -1,
  -82,
  -165,
  -247,
  -324,
  -391,
  -444,
  -478,
  -491,
  -480,
  -441,
  -373,
  -277,
  -152,
  0,
  175,
  372,
  584,
  808,
  1036,
  1263,
  1483,
  1689,
  1875,
  2035,
  2165,
  2261,
  2320,
  2340,
  2320,
  2261,
  2165,
  2035,
  1875,
  1689,
  1483,
  1263,
  1036,
  808,
  584,
  372,
  175,
  0,
  -152,
  -277,
  -373,
  -441,
  -480,
  -491,
  -478,
  -444,
  -391,
  -324,
  -247,
  -165,
  -82,
  -1,
  73,
  138,
  192,
  232,
  258,
  269,
  267,
  251,
  225,
  189,
  146,
  99,
  49,
  1,
  -45,
  -86,
  -120,
  -146,
  -164,
  -173,
  -172,
  -164,
  -147,
  -125,
  -97,
  -67,
  -35,
  -3,
  28,
  55,
  78,
  96,
  108,
  115,
  115,
  109,
  99,
  84,
  66,
  45,
  24,
  2,
  -19,
  -37,
  -53,
  -66,
  -74,
  -79,
  -79,
  -76,
  -69,
  -59,
  -47,
  -33,
  -19,
  -4,
  10,
  23,
  34,
  42,
  48,
  51,
  52,
  50,
  45,
  39,
  31,
  22,
  13,
  3,
  -6,
  -15,
  -22,
  -28,
  -32,
  -34,
  -35,
  -34,
  -31,
  -28,
  -23,
  -17,
  -11,
  -5,
  1,
  7,
  11,
  15,
  18,
  20,
  20,
  20,
  19,
  16,
  13,
  10,
  6,
  3,
  -1,
  -4,
  -7,
  -10,
  -12,
  -13,
  -14,
  -14,
  -13,
  -12,
  -10,
  -8,
  -6,
  -4,
  -2,
  0,
  2,
  4,
  5,
  6,
  7,
  8,
  8,
  8,
  8,
  7,
  6,
  6,
  5,
  4,
  3,
  3,
  2,
  2
};

/*
sampling frequency: 64000 Hz

fixed point precision: 16 bits

* 0 Hz - 1800 Hz
  gain = 0
  desired attenuation = -80 dB
  actual attenuation = n/a

* 2300 Hz - 4000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 4700 Hz - 32000 Hz
  gain = 0
  desired attenuation = -80 dB
  actual attenuation = n/a

*/

#define ROOFING_FILTER_TAP_NUM_2 321
#define ROOFING_FILTER_PRECISION_2 15

static int roofingFilterTaps2[ROOFING_FILTER_TAP_NUM_2] =
{
  13,
  7,
  7,
  6,
  3,
  -2,
  -8,
  -15,
  -21,
  -27,
  -30,
  -30,
  -26,
  -18,
  -6,
  8,
  23,
  37,
  49,
  56,
  57,
  52,
  41,
  25,
  5,
  -15,
  -33,
  -47,
  -55,
  -57,
  -52,
  -42,
  -28,
  -13,
  1,
  11,
  16,
  16,
  11,
  2,
  -8,
  -16,
  -20,
  -18,
  -9,
  6,
  26,
  47,
  67,
  81,
  87,
  83,
  68,
  43,
  11,
  -24,
  -57,
  -84,
  -101,
  -106,
  -99,
  -81,
  -56,
  -28,
  -1,
  20,
  32,
  33,
  24,
  9,
  -9,
  -25,
  -34,
  -32,
  -18,
  9,
  45,
  85,
  123,
  152,
  165,
  159,
  133,
  88,
  29,
  -37,
  -100,
  -154,
  -190,
  -204,
  -194,
  -164,
  -118,
  -65,
  -14,
  28,
  54,
  60,
  48,
  22,
  -11,
  -41,
  -59,
  -58,
  -33,
  15,
  82,
  159,
  233,
  292,
  323,
  319,
  275,
  192,
  80,
  -49,
  -179,
  -293,
  -376,
  -417,
  -411,
  -361,
  -275,
  -170,
  -62,
  32,
  95,
  120,
  105,
  56,
  -12,
  -80,
  -126,
  -132,
  -84,
  23,
  184,
  380,
  586,
  768,
  892,
  926,
  850,
  653,
  343,
  -57,
  -507,
  -959,
  -1357,
  -1649,
  -1791,
  -1755,
  -1530,
  -1132,
  -594,
  30,
  674,
  1270,
  1751,
  2064,
  2172,
  2064,
  1751,
  1270,
  674,
  30,
  -594,
  -1132,
  -1530,
  -1755,
  -1791,
  -1649,
  -1357,
  -959,
  -507,
  -57,
  343,
  653,
  850,
  926,
  892,
  768,
  586,
  380,
  184,
  23,
  -84,
  -132,
  -126,
  -80,
  -12,
  56,
  105,
  120,
  95,
  32,
  -62,
  -170,
  -275,
  -361,
  -411,
  -417,
  -376,
  -293,
  -179,
  -49,
  80,
  192,
  275,
  319,
  323,
  292,
  233,
  159,
  82,
  15,
  -33,
  -58,
  -59,
  -41,
  -11,
  22,
  48,
  60,
  54,
  28,
  -14,
  -65,
  -118,
  -164,
  -194,
  -204,
  -190,
  -154,
  -100,
  -37,
  29,
  88,
  133,
  159,
  165,
  152,
  123,
  85,
  45,
  9,
  -18,
  -32,
  -34,
  -25,
  -9,
  9,
  24,
  33,
  32,
  20,
  -1,
  -28,
  -56,
  -81,
  -99,
  -106,
  -101,
  -84,
  -57,
  -24,
  11,
  43,
  68,
  83,
  87,
  81,
  67,
  47,
  26,
  6,
  -9,
  -18,
  -20,
  -16,
  -8,
  2,
  11,
  16,
  16,
  11,
  1,
  -13,
  -28,
  -42,
  -52,
  -57,
  -55,
  -47,
  -33,
  -15,
  5,
  25,
  41,
  52,
  57,
  56,
  49,
  37,
  23,
  8,
  -6,
  -18,
  -26,
  -30,
  -30,
  -27,
  -21,
  -15,
  -8,
  -2,
  3,
  6,
  7,
  7,
  13
};

/*
sampling frequency: 64000 Hz

fixed point precision: 16 bits

* 0 Hz - 3800 Hz
  gain = 0
  desired attenuation = -80 dB
  actual attenuation = n/a

* 4300 Hz - 6000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 6700 Hz - 32000 Hz
  gain = 0
  desired attenuation = -80 dB
  actual attenuation = n/a

*/

#define ROOFING_FILTER_TAP_NUM_3 321
#define ROOFING_FILTER_PRECISION_3 15

static int roofingFilterTaps3[ROOFING_FILTER_TAP_NUM_3] =
{
  12,
  5,
  2,
  -3,
  -10,
  -17,
  -20,
  -18,
  -10,
  3,
  18,
  31,
  36,
  32,
  18,
  -4,
  -27,
  -46,
  -54,
  -48,
  -28,
  1,
  31,
  53,
  62,
  54,
  31,
  1,
  -27,
  -47,
  -52,
  -43,
  -23,
  -1,
  16,
  24,
  21,
  10,
  -1,
  -6,
  -3,
  10,
  26,
  37,
  37,
  21,
  -8,
  -43,
  -72,
  -85,
  -73,
  -39,
  11,
  61,
  98,
  110,
  91,
  48,
  -7,
  -58,
  -89,
  -94,
  -72,
  -35,
  5,
  33,
  41,
  31,
  11,
  -7,
  -12,
  2,
  30,
  62,
  81,
  73,
  35,
  -28,
  -98,
  -152,
  -169,
  -138,
  -64,
  35,
  132,
  197,
  210,
  167,
  80,
  -24,
  -115,
  -168,
  -169,
  -126,
  -57,
  11,
  54,
  63,
  40,
  5,
  -20,
  -16,
  23,
  86,
  146,
  173,
  144,
  53,
  -81,
  -221,
  -322,
  -344,
  -270,
  -111,
  92,
  283,
  407,
  424,
  329,
  151,
  -59,
  -238,
  -337,
  -336,
  -247,
  -112,
  15,
  91,
  96,
  44,
  -25,
  -58,
  -18,
  103,
  272,
  424,
  480,
  382,
  117,
  -270,
  -677,
  -975,
  -1047,
  -826,
  -330,
  338,
  1005,
  1479,
  1603,
  1303,
  625,
  -277,
  -1169,
  -1810,
  -2009,
  -1693,
  -925,
  106,
  1133,
  1884,
  2159,
  1884,
  1133,
  106,
  -925,
  -1693,
  -2009,
  -1810,
  -1169,
  -277,
  625,
  1303,
  1603,
  1479,
  1005,
  338,
  -330,
  -826,
  -1047,
  -975,
  -677,
  -270,
  117,
  382,
  480,
  424,
  272,
  103,
  -18,
  -58,
  -25,
  44,
  96,
  91,
  15,
  -112,
  -247,
  -336,
  -337,
  -238,
  -59,
  151,
  329,
  424,
  407,
  283,
  92,
  -111,
  -270,
  -344,
  -322,
  -221,
  -81,
  53,
  144,
  173,
  146,
  86,
  23,
  -16,
  -20,
  5,
  40,
  63,
  54,
  11,
  -57,
  -126,
  -169,
  -168,
  -115,
  -24,
  80,
  167,
  210,
  197,
  132,
  35,
  -64,
  -138,
  -169,
  -152,
  -98,
  -28,
  35,
  73,
  81,
  62,
  30,
  2,
  -12,
  -7,
  11,
  31,
  41,
  33,
  5,
  -35,
  -72,
  -94,
  -89,
  -58,
  -7,
  48,
  91,
  110,
  98,
  61,
  11,
  -39,
  -73,
  -85,
  -72,
  -43,
  -8,
  21,
  37,
  37,
  26,
  10,
  -3,
  -6,
  -1,
  10,
  21,
  24,
  16,
  -1,
  -23,
  -43,
  -52,
  -47,
  -27,
  1,
  31,
  54,
  62,
  53,
  31,
  1,
  -28,
  -48,
  -54,
  -46,
  -27,
  -4,
  18,
  32,
  36,
  31,
  18,
  3,
  -10,
  -18,
  -20,
  -17,
  -10,
  -3,
  2,
  5,
  12
};
#endif

/*
sampling frequency: 8000 Hz

fixed point precision: 16 bits

* 0 Hz - 250 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

* 450 Hz - 950 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 1150 Hz - 4000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

*/

#define CW_FILTER_TAP_NUM_1 64
#define CW_FILTER_PRECISION_1 15

static int cwFilterTaps1[CW_FILTER_TAP_NUM_1] =
{
  162,
  -149,
  -185,
  -197,
  -151,
  -65,
  -6,
  -35,
  -160,
  -304,
  -340,
  -165,
  215,
  661,
  958,
  936,
  586,
  92,
  -256,
  -243,
  114,
  527,
  572,
  -60,
  -1325,
  -2756,
  -3632,
  -3321,
  -1637,
  990,
  3644,
  5303,
  5303,
  3644,
  990,
  -1637,
  -3321,
  -3632,
  -2756,
  -1325,
  -60,
  572,
  527,
  114,
  -243,
  -256,
  92,
  586,
  936,
  958,
  661,
  215,
  -165,
  -340,
  -304,
  -160,
  -35,
  -6,
  -65,
  -151,
  -197,
  -185,
  -149,
  162
};

/*
sampling frequency: 8000 Hz

fixed point precision: 16 bits

* 0 Hz - 250 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

* 450 Hz - 950 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 1150 Hz - 4000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

*/

#define CW_FILTER_TAP_NUM_2 128
#define CW_FILTER_PRECISION_2 15

static int cwFilterTaps2[CW_FILTER_TAP_NUM_2] =
{
  17,
  9,
  6,
  1,
  0,
  6,
  18,
  29,
  26,
  6,
  -28,
  -61,
  -76,
  -63,
  -30,
  2,
  11,
  -11,
  -47,
  -67,
  -42,
  31,
  125,
  192,
  192,
  125,
  29,
  -35,
  -26,
  47,
  124,
  126,
  11,
  -190,
  -383,
  -461,
  -374,
  -168,
  37,
  115,
  24,
  -158,
  -266,
  -152,
  205,
  666,
  994,
  988,
  628,
  103,
  -279,
  -283,
  77,
  506,
  563,
  -71,
  -1353,
  -2806,
  -3692,
  -3372,
  -1657,
  1012,
  3706,
  5389,
  5389,
  3706,
  1012,
  -1657,
  -3372,
  -3692,
  -2806,
  -1353,
  -71,
  563,
  506,
  77,
  -283,
  -279,
  103,
  628,
  988,
  994,
  666,
  205,
  -152,
  -266,
  -158,
  24,
  115,
  37,
  -168,
  -374,
  -461,
  -383,
  -190,
  11,
  126,
  124,
  47,
  -26,
  -35,
  29,
  125,
  192,
  192,
  125,
  31,
  -42,
  -67,
  -47,
  -11,
  11,
  2,
  -30,
  -63,
  -76,
  -61,
  -28,
  6,
  26,
  29,
  18,
  6,
  0,
  1,
  6,
  9,
  17
};

/*
sampling frequency: 8000 Hz

fixed point precision: 16 bits

* 0 Hz - 400 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

* 550 Hz - 850 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 1000 Hz - 4000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define CW_FILTER_TAP_NUM_3 128
#define CW_FILTER_PRECISION_3 15

static int cwFilterTaps3[CW_FILTER_TAP_NUM_3] =
{
  75,
  -9,
  71,
  49,
  -36,
  -93,
  -92,
  -62,
  -33,
  -13,
  -2,
  -5,
  -23,
  -43,
  -42,
  -1,
  72,
  149,
  190,
  167,
  78,
  -49,
  -166,
  -227,
  -209,
  -128,
  -30,
  33,
  30,
  -30,
  -95,
  -103,
  -11,
  169,
  371,
  497,
  465,
  255,
  -75,
  -405,
  -603,
  -596,
  -400,
  -123,
  90,
  134,
  5,
  -188,
  -266,
  -80,
  397,
  1024,
  1521,
  1584,
  1030,
  -94,
  -1477,
  -2642,
  -3113,
  -2615,
  -1209,
  705,
  2512,
  3603,
  3603,
  2512,
  705,
  -1209,
  -2615,
  -3113,
  -2642,
  -1477,
  -94,
  1030,
  1584,
  1521,
  1024,
  397,
  -80,
  -266,
  -188,
  5,
  134,
  90,
  -123,
  -400,
  -596,
  -603,
  -405,
  -75,
  255,
  465,
  497,
  371,
  169,
  -11,
  -103,
  -95,
  -30,
  30,
  33,
  -30,
  -128,
  -209,
  -227,
  -166,
  -49,
  78,
  167,
  190,
  149,
  72,
  -1,
  -42,
  -43,
  -23,
  -5,
  -2,
  -13,
  -33,
  -62,
  -92,
  -93,
  -36,
  49,
  71,
  -9,
  75
};

#define CW_FILTER_TAP_NUM_4 257
#define CW_FILTER_PRECISION_4 19

static int cwFilterTaps4[CW_FILTER_TAP_NUM_4] =
{
0,
0,
1,
1,
1,
0,
-1,
-2,
-2,
-2,
-1,
0,
2,
4,
5,
4,
2,
-2,
-5,
-8,
-8,
-6,
-1,
4,
9,
11,
10,
6,
0,
-6,
-11,
-12,
-10,
-5,
2,
7,
9,
9,
6,
2,
-1,
-1,
0,
2,
3,
0,
-7,
-15,
-21,
-22,
-13,
4,
27,
48,
59,
51,
22,
-23,
-73,
-110,
-118,
-85,
-13,
85,
368,
583,
622,
421,
0,
-529,
-989,
-1200,
-1044,
-524,
225,
969,
1458,
1513,
1103,
360,
-462,
-1085,
-1311,
-1105,
-607,
-66,
268,
269,
-2,
-307,
-345,
96,
998,
2058,
2767,
2598,
1258,
-1116,
-3896,
-6106,
-6736,
-5141,
-1372,
3699,
8550,
11451,
11064,
6978,
0,
-7958,
-14398,
-17023,
-14542,
-7215,
3077,
13314,
20237,
21406,
16055,
5456,
-7377,
-18565,
-24574,
-23359,
-15069,
-2066,
11768,
22231,
26117,
22231,
11768,
-2066,
-15069,
-23359,
-24574,
-18565,
-7377,
5456,
16055,
21406,
20237,
13314,
3077,
-7215,
-14542,
-17023,
-14398,
-7958,
0,
6978,
11064,
11451,
8550,
3699,
-1372,
-5141,
-6736,
-6106,
-3896,
-1116,
1258,
2598,
2767,
2058,
998,
96,
-345,
-307,
-2,
269,
268,
-66,
-607,
-1105,
-1311,
-1085,
-462,
360,
1103,
1513,
1458,
969,
225,
-524,
-1044,
-1200,
-989,
-529,
0,
421,
622,
583,
368,
85,
-13,
-85,
-118,
-110,
-73,
-23,
22,
51,
59,
48,
27,
4,
-13,
-22,
-21,
-15,
-7,
0,
3,
2,
0,
-1,
-1,
2,
6,
9,
9,
7,
2,
-5,
-10,
-12,
-11,
-6,
0,
6,
10,
11,
9,
4,
-1,
-6,
-8,
-8,
-5,
-2,
2,
4,
5,
4,
2,
0,
-1,
-2,
-2,
-2,
-1,
0,
1,
1,
1,
0,
0,
};

/*
sampling frequency: 8000 Hz

fixed point precision: 16 bits

* 0 Hz - 300 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

* 400 Hz - 2700 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 2800 Hz - 4000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

*/

#define SSB_FILTER_TAP_NUM_2700 251
#define SSB_FILTER_PRECISION_2700 15

static int ssbFilterTaps2700[SSB_FILTER_TAP_NUM_2700] =
{
  6,
  22,
  16,
  -25,
  -10,
  0,
  -19,
  -10,
  -2,
  -20,
  -5,
  3,
  -14,
  7,
  12,
  -5,
  22,
  20,
  0,
  30,
  19,
  -4,
  26,
  3,
  -21,
  13,
  -20,
  -39,
  1,
  -38,
  -45,
  4,
  -39,
  -30,
  26,
  -22,
  3,
  59,
  0,
  35,
  82,
  5,
  48,
  75,
  -21,
  30,
  36,
  -72,
  -4,
  -15,
  -120,
  -25,
  -45,
  -134,
  -4,
  -34,
  -99,
  63,
  14,
  -31,
  146,
  60,
  26,
  198,
  58,
  30,
  183,
  -15,
  -28,
  105,
  -138,
  -110,
  15,
  -246,
  -149,
  -19,
  -276,
  -89,
  42,
  -205,
  68,
  169,
  -81,
  251,
  277,
  0,
  360,
  268,
  -45,
  327,
  99,
  -223,
  175,
  -172,
  -443,
  23,
  -406,
  -552,
  19,
  -466,
  -427,
  248,
  -307,
  -68,
  654,
  -44,
  369,
  1033,
  83,
  640,
  1122,
  -187,
  542,
  746,
  -999,
  59,
  -74,
  -2319,
  -528,
  -1090,
  -4078,
  -471,
  -1926,
  -7555,
  5827,
  19596,
  5827,
  -7555,
  -1926,
  -471,
  -4078,
  -1090,
  -528,
  -2319,
  -74,
  59,
  -999,
  746,
  542,
  -187,
  1122,
  640,
  83,
  1033,
  369,
  -44,
  654,
  -68,
  -307,
  248,
  -427,
  -466,
  19,
  -552,
  -406,
  23,
  -443,
  -172,
  175,
  -223,
  99,
  327,
  -45,
  268,
  360,
  0,
  277,
  251,
  -81,
  169,
  68,
  -205,
  42,
  -89,
  -276,
  -19,
  -149,
  -246,
  15,
  -110,
  -138,
  105,
  -28,
  -15,
  183,
  30,
  58,
  198,
  26,
  60,
  146,
  -31,
  14,
  63,
  -99,
  -34,
  -4,
  -134,
  -45,
  -25,
  -120,
  -15,
  -4,
  -72,
  36,
  30,
  -21,
  75,
  48,
  5,
  82,
  35,
  0,
  59,
  3,
  -22,
  26,
  -30,
  -39,
  4,
  -45,
  -38,
  1,
  -39,
  -20,
  13,
  -21,
  3,
  26,
  -4,
  19,
  30,
  0,
  20,
  22,
  -5,
  12,
  7,
  -14,
  3,
  -5,
  -20,
  -2,
  -10,
  -19,
  0,
  -10,
  -25,
  16,
  22,
  6
};

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 8000 Hz

fixed point precision: 16 bits

* 0 Hz - 450 Hz
  gain = 0
  desired attenuation = -50 dB
  actual attenuation = n/a

* 550 Hz - 850 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 950 Hz - 4000 Hz
  gain = 0
  desired attenuation = -50 dB
  actual attenuation = n/a

*/

#define CW_FILTER_TAP_NUM_5 280
#define CW_FILTER_PRECISION_5 15

static int cwFilterTaps5[CW_FILTER_TAP_NUM_5] =
{
  5,
  -6,
  3,
  6,
  4,
  0,
  -5,
  -10,
  -12,
  -11,
  -6,
  0,
  6,
  10,
  10,
  7,
  2,
  -1,
  -2,
  1,
  4,
  5,
  2,
  -5,
  -15,
  -23,
  -24,
  -16,
  -1,
  17,
  31,
  35,
  29,
  14,
  -3,
  -16,
  -20,
  -15,
  -6,
  0,
  -2,
  -13,
  -26,
  -34,
  -28,
  -6,
  26,
  57,
  74,
  68,
  39,
  -3,
  -44,
  -67,
  -67,
  -46,
  -16,
  7,
  13,
  2,
  -15,
  -23,
  -11,
  25,
  72,
  109,
  114,
  77,
  6,
  -77,
  -141,
  -162,
  -131,
  -62,
  16,
  73,
  90,
  69,
  30,
  2,
  7,
  46,
  99,
  129,
  108,
  24,
  -100,
  -220,
  -286,
  -263,
  -151,
  12,
  169,
  261,
  262,
  181,
  66,
  -27,
  -57,
  -21,
  44,
  79,
  37,
  -91,
  -264,
  -403,
  -426,
  -292,
  -23,
  298,
  553,
  642,
  528,
  255,
  -67,
  -313,
  -397,
  -316,
  -148,
  -17,
  -25,
  -195,
  -443,
  -610,
  -532,
  -126,
  542,
  1262,
  1738,
  1706,
  1058,
  -93,
  -1415,
  -2466,
  -2845,
  -2351,
  -1072,
  620,
  2194,
  3138,
  3138,
  2194,
  620,
  -1072,
  -2351,
  -2845,
  -2466,
  -1415,
  -93,
  1058,
  1706,
  1738,
  1262,
  542,
  -126,
  -532,
  -610,
  -443,
  -195,
  -25,
  -17,
  -148,
  -316,
  -397,
  -313,
  -67,
  255,
  528,
  642,
  553,
  298,
  -23,
  -292,
  -426,
  -403,
  -264,
  -91,
  37,
  79,
  44,
  -21,
  -57,
  -27,
  66,
  181,
  262,
  261,
  169,
  12,
  -151,
  -263,
  -286,
  -220,
  -100,
  24,
  108,
  129,
  99,
  46,
  7,
  2,
  30,
  69,
  90,
  73,
  16,
  -62,
  -131,
  -162,
  -141,
  -77,
  6,
  77,
  114,
  109,
  72,
  25,
  -11,
  -23,
  -15,
  2,
  13,
  7,
  -16,
  -46,
  -67,
  -67,
  -44,
  -3,
  39,
  68,
  74,
  57,
  26,
  -6,
  -28,
  -34,
  -26,
  -13,
  -2,
  0,
  -6,
  -15,
  -20,
  -16,
  -3,
  14,
  29,
  35,
  31,
  17,
  -1,
  -16,
  -24,
  -23,
  -15,
  -5,
  2,
  5,
  4,
  1,
  -2,
  -1,
  2,
  7,
  10,
  10,
  6,
  0,
  -6,
  -11,
  -12,
  -10,
  -5,
  0,
  4,
  6,
  3,
  -6,
  5
};

// Number of filters
#define NUM_FILTERS 7

// The different filters available to the user
const struct sFir
{
  const int*        taps;
  const int         numTaps;
  const int         precision;
  const bool        binaural;
  const char* const text;
}
filters[NUM_FILTERS]=
{
  { NULL,              0,                       0,                         false, "Off" },
  { cwFilterTaps1,     CW_FILTER_TAP_NUM_1,     CW_FILTER_PRECISION_1,     true,  "Binaural" },
  { cwFilterTaps2,     CW_FILTER_TAP_NUM_2,     CW_FILTER_PRECISION_2,     false, "500Hz" },
  { cwFilterTaps3,     CW_FILTER_TAP_NUM_3,     CW_FILTER_PRECISION_3,     false, "300Hz 1" },
  { cwFilterTaps5,     CW_FILTER_TAP_NUM_5,     CW_FILTER_PRECISION_5,     false, "300Hz 2" },
  { cwFilterTaps4,     CW_FILTER_TAP_NUM_4,     CW_FILTER_PRECISION_4,     false, "CW 4" },
  { ssbFilterTaps2700, SSB_FILTER_TAP_NUM_2700, SSB_FILTER_PRECISION_2700, false, "2700Hz" },
};

// Size of the buffer so must be at least as large as the
// biggest number of taps above
#define FILTER_TAP_NUM 280
