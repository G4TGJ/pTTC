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
#if 0
#define DECIMATE_16_08_FILTER_TAP_NUM 5
#define DECIMATE_16_08_FILTER_PRECISION 15

static int decimate1608FilterTaps[DECIMATE_16_08_FILTER_TAP_NUM] =
{
  32768,
  0,
  0,
  0,
  0,
};

#else
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
#endif
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

/*
sampling frequency: 8000 Hz

fixed point precision: 16 bits

* 0 Hz - 200 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 400 Hz - 4000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

*/

#define CW_FILTER_TAP_NUM_400 123
#define CW_FILTER_PRECISION_400 15

static int cwFilterTaps400[CW_FILTER_TAP_NUM_400] =
{
  19,
  7,
  7,
  6,
  4,
  1,
  -4,
  -9,
  -16,
  -24,
  -31,
  -39,
  -46,
  -51,
  -54,
  -54,
  -51,
  -43,
  -32,
  -16,
  3,
  26,
  50,
  76,
  101,
  124,
  143,
  155,
  161,
  157,
  142,
  117,
  81,
  34,
  -22,
  -84,
  -151,
  -219,
  -283,
  -340,
  -385,
  -414,
  -422,
  -405,
  -362,
  -288,
  -185,
  -51,
  112,
  301,
  511,
  737,
  974,
  1214,
  1449,
  1673,
  1878,
  2056,
  2201,
  2309,
  2375,
  2398,
  2375,
  2309,
  2201,
  2056,
  1878,
  1673,
  1449,
  1214,
  974,
  737,
  511,
  301,
  112,
  -51,
  -185,
  -288,
  -362,
  -405,
  -422,
  -414,
  -385,
  -340,
  -283,
  -219,
  -151,
  -84,
  -22,
  34,
  81,
  117,
  142,
  157,
  161,
  155,
  143,
  124,
  101,
  76,
  50,
  26,
  3,
  -16,
  -32,
  -43,
  -51,
  -54,
  -54,
  -51,
  -46,
  -39,
  -31,
  -24,
  -16,
  -9,
  -4,
  1,
  4,
  6,
  7,
  7,
  19
};

/*
sampling frequency: 8000 Hz

fixed point precision: 16 bits

* 0 Hz - 400 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 600 Hz - 4000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

*/

#define CW_FILTER_TAP_NUM_800 119
#define CW_FILTER_PRECISION_800 15

static int cwFilterTaps800[CW_FILTER_TAP_NUM_800] =
{
  -13,
  7,
  11,
  17,
  24,
  29,
  32,
  32,
  26,
  15,
  0,
  -19,
  -38,
  -54,
  -65,
  -66,
  -57,
  -37,
  -7,
  30,
  68,
  101,
  123,
  129,
  115,
  80,
  26,
  -40,
  -110,
  -173,
  -217,
  -233,
  -213,
  -155,
  -64,
  51,
  175,
  290,
  375,
  413,
  389,
  298,
  144,
  -60,
  -290,
  -514,
  -696,
  -800,
  -793,
  -651,
  -363,
  65,
  615,
  1251,
  1925,
  2584,
  3170,
  3632,
  3927,
  4029,
  3927,
  3632,
  3170,
  2584,
  1925,
  1251,
  615,
  65,
  -363,
  -651,
  -793,
  -800,
  -696,
  -514,
  -290,
  -60,
  144,
  298,
  389,
  413,
  375,
  290,
  175,
  51,
  -64,
  -155,
  -213,
  -233,
  -217,
  -173,
  -110,
  -40,
  26,
  80,
  115,
  129,
  123,
  101,
  68,
  30,
  -7,
  -37,
  -57,
  -66,
  -65,
  -54,
  -38,
  -19,
  0,
  15,
  26,
  32,
  32,
  29,
  24,
  17,
  11,
  7,
  -13
};

/*
sampling frequency: 8000 Hz

fixed point precision: 16 bits

* 0 Hz - 100 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 200 Hz - 4000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

*/

#define CW_FILTER_TAP_NUM_200 243
#define CW_FILTER_PRECISION_200 15

static int cwFilterTaps200[CW_FILTER_TAP_NUM_200] =
{
  18,
  3,
  3,
  3,
  3,
  2,
  2,
  2,
  1,
  0,
  -1,
  -2,
  -3,
  -5,
  -6,
  -8,
  -10,
  -12,
  -14,
  -16,
  -17,
  -19,
  -21,
  -22,
  -24,
  -25,
  -26,
  -26,
  -27,
  -26,
  -26,
  -25,
  -23,
  -21,
  -18,
  -15,
  -12,
  -8,
  -3,
  2,
  7,
  13,
  19,
  25,
  32,
  38,
  44,
  50,
  56,
  62,
  67,
  71,
  74,
  77,
  79,
  80,
  79,
  78,
  75,
  70,
  65,
  58,
  49,
  40,
  29,
  16,
  3,
  -11,
  -27,
  -43,
  -59,
  -76,
  -93,
  -110,
  -126,
  -142,
  -156,
  -170,
  -182,
  -192,
  -201,
  -207,
  -210,
  -210,
  -208,
  -202,
  -193,
  -180,
  -164,
  -143,
  -119,
  -92,
  -60,
  -25,
  14,
  57,
  102,
  151,
  202,
  256,
  311,
  369,
  428,
  487,
  547,
  607,
  666,
  725,
  781,
  836,
  889,
  938,
  985,
  1027,
  1066,
  1100,
  1130,
  1154,
  1173,
  1187,
  1195,
  1198,
  1195,
  1187,
  1173,
  1154,
  1130,
  1100,
  1066,
  1027,
  985,
  938,
  889,
  836,
  781,
  725,
  666,
  607,
  547,
  487,
  428,
  369,
  311,
  256,
  202,
  151,
  102,
  57,
  14,
  -25,
  -60,
  -92,
  -119,
  -143,
  -164,
  -180,
  -193,
  -202,
  -208,
  -210,
  -210,
  -207,
  -201,
  -192,
  -182,
  -170,
  -156,
  -142,
  -126,
  -110,
  -93,
  -76,
  -59,
  -43,
  -27,
  -11,
  3,
  16,
  29,
  40,
  49,
  58,
  65,
  70,
  75,
  78,
  79,
  80,
  79,
  77,
  74,
  71,
  67,
  62,
  56,
  50,
  44,
  38,
  32,
  25,
  19,
  13,
  7,
  2,
  -3,
  -8,
  -12,
  -15,
  -18,
  -21,
  -23,
  -25,
  -26,
  -26,
  -27,
  -26,
  -26,
  -25,
  -24,
  -22,
  -21,
  -19,
  -17,
  -16,
  -14,
  -12,
  -10,
  -8,
  -6,
  -5,
  -3,
  -2,
  -1,
  0,
  1,
  2,
  2,
  2,
  3,
  3,
  3,
  3,
  18
};

/*
sampling frequency: 8000 Hz

fixed point precision: 16 bits

* 0 Hz - 500 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = n/a

* 800 Hz - 4000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = n/a

*/

#define CW_FILTER_TAP_NUM_1000 87
#define CW_FILTER_PRECISION_1000 15

static int cwFilterTaps1000[CW_FILTER_TAP_NUM_1000] =
{
  16,
  19,
  24,
  24,
  17,
  2,
  -19,
  -42,
  -60,
  -67,
  -56,
  -26,
  20,
  73,
  118,
  140,
  128,
  76,
  -11,
  -115,
  -211,
  -269,
  -264,
  -184,
  -36,
  155,
  344,
  478,
  507,
  399,
  153,
  -195,
  -574,
  -890,
  -1038,
  -929,
  -510,
  219,
  1200,
  2321,
  3434,
  4377,
  5009,
  5232,
  5009,
  4377,
  3434,
  2321,
  1200,
  219,
  -510,
  -929,
  -1038,
  -890,
  -574,
  -195,
  153,
  399,
  507,
  478,
  344,
  155,
  -36,
  -184,
  -264,
  -269,
  -211,
  -115,
  -11,
  76,
  128,
  140,
  118,
  73,
  20,
  -26,
  -56,
  -67,
  -60,
  -42,
  -19,
  2,
  17,
  24,
  24,
  19,
  16
};

// Number of filters
#define NUM_FILTERS 5

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
#if 1
  { cwFilterTaps200,   CW_FILTER_TAP_NUM_200,   CW_FILTER_PRECISION_200,   false,  "200Hz" },
  { cwFilterTaps400,   CW_FILTER_TAP_NUM_400,   CW_FILTER_PRECISION_400,   false,  "400Hz" },
  { cwFilterTaps800,   CW_FILTER_TAP_NUM_800,   CW_FILTER_PRECISION_800,   false,  "800Hz" },
  { cwFilterTaps1000,  CW_FILTER_TAP_NUM_1000,  CW_FILTER_PRECISION_1000,  false,  "1000Hz" },
#else
  { cwFilterTaps1,     CW_FILTER_TAP_NUM_1,     CW_FILTER_PRECISION_1,     true,  "Binaural" },
  { cwFilterTaps2,     CW_FILTER_TAP_NUM_2,     CW_FILTER_PRECISION_2,     false, "500Hz" },
  { cwFilterTaps3,     CW_FILTER_TAP_NUM_3,     CW_FILTER_PRECISION_3,     false, "300Hz 1" },
  { cwFilterTaps5,     CW_FILTER_TAP_NUM_5,     CW_FILTER_PRECISION_5,     false, "300Hz 2" },
  { cwFilterTaps4,     CW_FILTER_TAP_NUM_4,     CW_FILTER_PRECISION_4,     false, "CW 4" },
  { ssbFilterTaps2700, SSB_FILTER_TAP_NUM_2700, SSB_FILTER_PRECISION_2700, false, "2700Hz" },
#endif
};

// Size of the buffer so must be at least as large as the
// biggest number of taps above and a power of 2
#define OUTPUT_BUFFER_LEN 512
