#define HILBERT_FILTER_1_TAP_NUM 31
#define HILBERT_FILTER_1_PRECISION 15

static int hilbertFilter1Taps[HILBERT_FILTER_1_TAP_NUM] = 
{
    112,
    0,
    193,
    0,
    442,
    0,
    925,
    0,
    1758,
    0,
    3223,
    0,
    6363,
    0,
    20718,
    0,
    -20718,
    0,
    -6363,
    0,
    -3223,
    0,
    -1758,
    0,
    -925,
    0,
    -442,
    0,
    -193,
    0,
    -112
};

#define HILBERT_FILTER_2_TAP_NUM 101
#define HILBERT_FILTER_2_PRECISION 15

static int hilbertFilter2Taps[HILBERT_FILTER_2_TAP_NUM] = 
{
    0,
    -34,
    0,
    -39,
    0,
    -48,
    0,
    -60,
    0,
    -77,
    0,
    -99,
    0,
    -127,
    0,
    -161,
    0,
    -201,
    0,
    -249,
    0,
    -306,
    0,
    -373,
    0,
    -451,
    0,
    -542,
    0,
    -650,
    0,
    -779,
    0,
    -935,
    0,
    -1127,
    0,
    -1372,
    0,
    -1696,
    0,
    -2152,
    0,
    -2850,
    0,
    -4078,
    0,
    -6897,
    0,
    -20842,
    0,
    20842,
    0,
    6897,
    0,
    4078,
    0,
    2850,
    0,
    2152,
    0,
    1696,
    0,
    1372,
    0,
    1127,
    0,
    935,
    0,
    779,
    0,
    650,
    0,
    542,
    0,
    451,
    0,
    373,
    0,
    306,
    0,
    249,
    0,
    201,
    0,
    161,
    0,
    127,
    0,
    99,
    0,
    77,
    0,
    60,
    0,
    48,
    0,
    39,
    0,
    34,
    0
};

#define HILBERT_FILTER_3_TAP_NUM 35
#define HILBERT_FILTER_3_PRECISION 12

static int hilbertFilter3Taps[HILBERT_FILTER_3_TAP_NUM] = 
{
    -9,
    0,
    -23,
    0,
    -47,
    0,
    -88,
    0,
    -152,
    0,
    -255,
    0,
    -431,
    0,
    -812,
    0,
    -2588,
    0,
    2588,
    0,
    812,
    0,
    431,
    0,
    255,
    0,
    152,
    0,
    88,
    0,
    47,
    0,
    23,
    0,
    9
};

// Number of hilbert filters
#define NUM_HILBERT_FILTERS 4

// The different hilber filters available to the user
const struct sHilbert
{
  const int*        taps;
  const int         numTaps;
  const int         precision;
  const char* const text;
}
hilbertFilters[NUM_FILTERS]=
{
  { NULL,               0,                        0,                          "Hilbert Off" },
  { hilbertFilter1Taps, HILBERT_FILTER_1_TAP_NUM, HILBERT_FILTER_1_PRECISION, "Hilbert 1"   },
  { hilbertFilter2Taps, HILBERT_FILTER_2_TAP_NUM, HILBERT_FILTER_2_PRECISION, "Hilbert 2"   },
  { hilbertFilter3Taps, HILBERT_FILTER_3_TAP_NUM, HILBERT_FILTER_3_PRECISION, "Hilbert 3"   },
};


// Size of the buffer so must be at least as large as the
// biggest number of taps above and a power of 2
#define HILBERT_FILTER_BUFFER_LEN 128
