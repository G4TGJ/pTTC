/*
FIR filters designed with
http://t-filter.engineerjs.com/
*/

/*
sampling frequency: 256000 Hz

fixed point precision: 16 bits

* 0 Hz - 11000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 117000 Hz - 128000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

*/

#define INTERPOLATE_FILTER_TAP_NUM 7
#define INTERPOLATE_FILTER_PRECISION 15

static int interpolateFilterTaps[INTERPOLATE_FILTER_TAP_NUM] =
{
  -1051,
  61,
  9246,
  16272,
  9246,
  61,
  -1051
};

#if 0
/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 128000 Hz

fixed point precision: 16 bits

* 0 Hz - 2000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 8000 Hz - 64000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define DECIMATE_128_64_FILTER_TAP_NUM 55
#define DECIMATE_128_64_FILTER_PRECISION 15

static int decimate12864FilterTaps[DECIMATE_128_64_FILTER_TAP_NUM] =
{
  122,
  23,
  13,
  -8,
  -39,
  -79,
  -126,
  -176,
  -225,
  -265,
  -290,
  -293,
  -266,
  -204,
  -101,
  44,
  232,
  460,
  721,
  1006,
  1305,
  1604,
  1889,
  2146,
  2362,
  2525,
  2627,
  2661,
  2627,
  2525,
  2362,
  2146,
  1889,
  1604,
  1305,
  1006,
  721,
  460,
  232,
  44,
  -101,
  -204,
  -266,
  -293,
  -290,
  -265,
  -225,
  -176,
  -126,
  -79,
  -39,
  -8,
  13,
  23,
  122
};

#else
#if 1
/*
sampling frequency: 128000 Hz

fixed point precision: 16 bits

* 0 Hz - 4000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 32000 Hz - 64000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define DECIMATE_128_64_FILTER_TAP_NUM 15
#define DECIMATE_128_64_FILTER_PRECISION 15

static int decimate12864FilterTaps[DECIMATE_128_64_FILTER_TAP_NUM] =
{
  -55,
  -345,
  -698,
  -444,
  1261,
  4428,
  7688,
  9085,
  7688,
  4428,
  1261,
  -444,
  -698,
  -345,
  -55
};

#else
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
#endif
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

#if 1
/*
sampling frequency: 64000 Hz

fixed point precision: 16 bits

* 0 Hz - 11000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 16000 Hz - 32000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define DECIMATE_64_32_8_FILTER_TAP_NUM 31
#define DECIMATE_64_32_8_FILTER_PRECISION 15

static int decimate64328FilterTaps[DECIMATE_64_32_8_FILTER_TAP_NUM] =
{
  156,
  -1,
  -253,
  -110,
  328,
  413,
  -339,
  -821,
  39,
  1372,
  709,
  -1877,
  -2536,
  2283,
  10081,
  13976,
  10081,
  2283,
  -2536,
  -1877,
  709,
  1372,
  39,
  -821,
  -339,
  413,
  328,
  -110,
  -253,
  -1,
  156
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
#else
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

#if 1
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

#define DECIMATE_16_08_FILTER_TAP_NUM 39
#define DECIMATE_16_08_FILTER_PRECISION 15

static int decimate1608FilterTaps[DECIMATE_16_08_FILTER_TAP_NUM] =
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
#else

/*
sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 250 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 1000 Hz - 8000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

*/

#define DECIMATE_16_8_FILTER_TAP_NUM 67
#define DECIMATE_16_8_FILTER_PRECISION 15

static int decimate168FilterTaps[DECIMATE_16_8_FILTER_TAP_NUM] =
{
  24,
  17,
  21,
  22,
  20,
  13,
  1,
  -18,
  -43,
  -74,
  -111,
  -150,
  -189,
  -224,
  -250,
  -262,
  -256,
  -225,
  -167,
  -77,
  45,
  202,
  389,
  605,
  842,
  1093,
  1349,
  1601,
  1836,
  2046,
  2221,
  2352,
  2433,
  2460,
  2433,
  2352,
  2221,
  2046,
  1836,
  1601,
  1349,
  1093,
  842,
  605,
  389,
  202,
  45,
  -77,
  -167,
  -225,
  -256,
  -262,
  -250,
  -224,
  -189,
  -150,
  -111,
  -74,
  -43,
  -18,
  1,
  13,
  20,
  22,
  21,
  17,
  24
};
#endif

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
#define LEFT_BUFFER_LEN 64
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
#define RIGHT_BUFFER_LEN 64
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
// biggest number of taps above and a power of 2
#define OUTPUT_BUFFER_LEN 512
