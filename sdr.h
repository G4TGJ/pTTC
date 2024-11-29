enum sdrSource
{
    sdrIMQ,
    sdrIPQ,
    sdrI,
    sdrQ,
    sdrIHilbert,
    sdrQHilbert,
    sdrIDelayed,
    sdrQDelayed,
    sdrSineWave,
    sdrSilence,
    sdrNum
};

extern enum sdrSource outputSource;

extern int iGain;
extern int qGain;
extern int iqGain;

extern bool applyGains;
extern bool applyRoofingFilter;
extern bool adjustPhase;

extern void ioSetPWMDiv( uint8_t div );
extern uint8_t ioGetPWMDiv( void );
