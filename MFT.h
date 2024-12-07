#ifndef MFT_h
#define MFT_h
#include "Hardware.h"

#define FILTER   0.1

#define SIGNATURE  0xAA55A5A5

#define ESC   27
#define ENQ   5

#define MINvoltage    0
#define MAXvoltage    50

#define MINfrequency  0
#define MAXfrequency  25000

#define MAXCMDLEN 200

// MAX5815 outputs
#define TW1ctrlCH     0
#define TW2ctrlCH     1
#define GRDctrlCH     2

// CPU ADC inputs
#define TW1monCH      A7
#define TW2monCH      A6
#define GRDmonCH      A8

#define TrigOut       9
#define Trig1         3
#define Trig2         2

enum TriggerFunction
{
  REV1_TF,
  REV2_TF,
  OPEN1_TF,
  OPEN2_TF,
  CMD_TF,
  CNT_TF,
  TWALT1_TF,
  TWALT2_TF,
  NA_TF
};

enum TriggerMode
{
  POS_MODE,
  NEG_MODE,
  CHANGE_MODE,
  NA_MODE
};

typedef struct
{
  uint32_t  count;              // Counter value
  uint32_t  tcount;             // Counter trigger level
  bool      resetOnTcount;      // Reset the count at threshold
  bool      triggerOnTcount;    // Generate an output trigger at threshold count
  bool      commandOnTcount;    // Execute command string at threshold count
} PulseCounter;

// TwaveSwitch data structure
typedef struct
{
  int16_t       Size;                   // This data structures size in bytes
  char          Name[20];               // Holds the board name, "TwaveSwitch"
  int8_t        Rev;                    // Holds the board revision number
  int           Baud;                   // Serial port baud rate, not error checked!
  uint8_t       MAX5815add;             // MAX5815 TWI address
  // DAC output channel and gains
  DACchan       TW1ctrl;
  DACchan       TW2ctrl;
  DACchan       GRDctrl;
  // ADC input channel and gains
  ADCchan       TW1mon;
  ADCchan       TW2mon;
  ADCchan       GRDmon;
  // Twave switch settings
  int           Mode;                   // Switch mode, 0 = generate Twave
  int           Freq;                   // Waveform frequency requested
  int           Afreq;                  // Actual generated frequency, per output
  bool          Enable;                 // If true the waveform is generated
  bool          Fwd[2];                 // If true the waveform is in forward direction, else reverse
  bool          Open[2];                // If true the waveform channen open mask is applied
  int           openMask[2];            // Bit mask that defines the open channels
  int           fwdPS[2];               // Forward phase shift in degrees
  int           revPS[2];               // Reverse phase shift in degrees
  uint16_t      bitPattern[2];          // Twave bit pattern
  uint16_t      twave[2][8];            // Twave waveform
  float         TWvoltage[2];           // Twave output voltage
  float         TWaltV[2];              // Twave output alternut voltage
  float         Guard;
  // Limits
  float         minTWV[2];              // Minimum twave voltage
  float         maxTWV[2];              // Maximum twave voltage
  float         minGuard;               // Minimum guard voltage
  float         maxGuard;               // Maximum guard voltage
  int           minFreq;                // Minimum frequency
  int           maxFreq;                // Maximum frequency
  //
  unsigned int  Signature;              // Must be 0xAA55A5A5 for valid data
} MFTdata;

extern bool MonitorFlag;

extern float TW1readback;
extern float TW2readback;
extern float GRDreadback;
extern char  Status[];
extern int   activeCS;
extern char  commandString[2][MAXCMDLEN];

extern PulseCounter pulseCounter;

extern int  clockFrequency;
extern char clockMode[];


// Prototypes...
void ReadAllSerial(void);
void ProcessSerial(bool scan = true);
void ReadADC(void);
void SetFrequency(char *value);
void SetFWDir(char *chan, char *fwd);
void GetFWDir(int ch);
void SetPattern(char *chan, char *ptrn);
void GetPattern(int ch);
void StopTwave(void);
void StartTwave(void);
void rtClockCyclsISR(void);
void MoveNcycles(int N);

void SetTWvoltage(char *chan, char *value);
void GetTWvoltage(int ch);
void SetTWAvoltage(char *chan, char *value);
void GetTWAvoltage(int ch);
void GetTWreadback(int ch);
void SetGRDvoltage(char *value);
void SetGRDvoltageAdj(char *value);
void SetTrigOut(char *value);

void Calibrate(void);

void setOpen(char *chan, char *val);
void getOpen(int ch);
void setOpenMask(char *chan, char *ptrn);
void getOpenMask(int ch);
void setFwdPS(char *chan, char *phase);
void getFwdPS(int ch);
void setRevPS(char *chan, char *phase);
void getRevPS(int ch);
void playCommandString(void);
void playCommandString1(void);
void playCommandString2(void);
void setTrig1(char *mode, char *function);
void setTrig2(char *mode, char *function);
void clearCounter(void);
void readTriggerInput(int ch);

void setMaximum(void);
void getMaximum(void);
void setMinimum(void);
void getMinimum(void);

void setActiveCMD(int ch);
void getActiveCMD(void);

void setClock(int freq);
void setClockFunction(char *func);

#endif
