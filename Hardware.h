#ifndef Hardware_h
#define Hardware_h

// DIO lines
#define LDAC     5
#define CLRDAC   4

#define LTCH     10
#define CLRMUX   14

// DAC channels
#define TW1V     0
#define TW2V     1
#define TWGRD    2

// M0 pin assigenments

typedef struct
{
  int8_t  Chan;                   // ADC channel number 0 through max channels for chip.
                                  // If MSB is set then this is a M0 ADC channel number
  float   m;                      // Calibration parameters to convert channel to engineering units
  float   b;                      // ADCcounts = m * value + b, value = (ADCcounts - b) / m
} ADCchan;

typedef struct
{
  int8_t  Chan;                   // DAC channel number 0 through max channels for chip
  float   m;                      // Calibration parameters to convert engineering to DAC counts
  float   b;                      // DACcounts = m * value + b, value = (DACcounts - b) / m
} DACchan;

// Function prototypes
int   GetADCvalue(int chan, int num);
float ReadADCchannel(ADCchan adch, int num=20);
float Counts2Value(int Counts, DACchan *DC);
float Counts2Value(int Counts, ADCchan *ad);
int   Value2Counts(float Value, DACchan *DC);
int   Value2Counts(float Value, ADCchan *ac);
void  MAX5815(int addr, int chn, int counts);

void ProgramFLASH(char * Faddress,char *Fsize);

void MAX14802(int TW1,int TW2, bool Latch = true);

#endif
