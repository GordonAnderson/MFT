//
// MFT
//
// The MFT uses a Teensyduino, you must use the Teensyduino IDE to develope the code and downlaod.
// Select Teensy 4.0 board.
//
// Revision history:
//
// 1.0, Aug 5, 2022
//    1.) Initial version
// 1.1, Aug 26, 2023
//    1.) Updated the set guard function to correct the non liearity at low guard settings
// 1.2, Oct 31, 2023
//    1.) Bug in set freq, no ACK was sent
//    2.) Add fwd/rev phase shift in degrees
//    3.) Add output disable for each channel, make this a mask and a open command
//    4.) Support += and -= on parameter values
//    5.) Define trigger command to execute
//    6.) Allow use of trigger
//        - Two input triggers, Trig1 and Trig2
//        - Trigger functions we need:
//          - Reverse
//          - disable output
//          - Trigger command
//    7.) Counting function
//        Count a trigger input and allow terminal count to do something:
//        - How many counters, 1 for now
//        - Generate a trigger output
//        - Trigger command
// 1.3, Nov 28, 2023
//    1.) Fixed counter bug that cause the enabled events to fire on every count over threshold
//    2.) Fixed missint break in case statement in trigger ISRs, the CMD option would also cause   
//        CNT event
// 1.4, Dec 10, 2023
//    1.) Added a second command string
//    2.) Added commands to select active command string
//        SCMD,num
//        GCMD
//    3.) Changed to push the command string onto the bottom of the command string
//    4.) Add if on select commands, set commands
//        STWV,1,>10|10|3
//        support >,<,>=,<=,=
//        if true and value after | is a number set this value if its a command
//        execute the command. Command must not have any commas, replace with underscore
//        if false the value after the second | is used
//    5.) Added programmable limits to:
//        TW voltage
//        Guard voltage
//        TW frequency
//        Only enter ch number for TWV
//        SMAX,type,chan,limit
//        SMAX,type,chan
//        SMIN,type,chan,limit
//        SMIN,type,chan
//    6.) Added a clock function
//        SCLOCK,freq
//        GCLOCK
//        freq 0 to 10000, 0 disables
//        SCLKFUN,NA, TRG or CNT
//        GCLKFUN
// 1.5, Jan 20, 2024
//    1.) Fixed counter command processing logic problem that could cause serial mute flag errors 
//        if counter processes a command with conditional that has a command executed in conditional.
//    2.) Reported error with = conditional, need to test
// 1.6, Feb 8, 2024
//    1.) Added alternut TW voltage capability
// 1.7, June 19, 2024
//    1.) Added SGRDA to set the guard voltage and use readback to lineraze below 5 volts
//    2.) Fixed commnuications issue when in the command execution loops
//
//
// Gordon Anderson
// GAA Custom Electronics, LLC
// 101904 Wiser Parkway, Suite 105
// Kennewick, WA  99338
// gaa@owt.com
// 509.628.6851
//
#include <Arduino.h>
#include <wiring_private.h>
#include <Thread.h>
#include <ThreadController.h>
#include <Adafruit_DotStar.h>

#include <SPI.h>
#include "Hardware.h"
#include "MFT.h"
#include "Errors.h"
#include "Serial.h"
#include <SerialBuffer.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include "AtomicBlock.h"

const char   Version[] PROGMEM = "MFT version 1.7, Jun 19, 2024";
MFTdata      mftdata;

int eeAddress = 0;

SerialBuffer sb;

// ThreadController that will control all threads
ThreadController control = ThreadController();
//Threads
Thread SystemThread = Thread();

IntervalTimer     clockTimer;
int  clockFrequency = 0;
char clockMode[5] = "NA";
void (*clockFunction)(bool high) = NULL;

int               activeCS = 0;
char              commandString[2][MAXCMDLEN] = {"STWV,1,>25|ETRGCMD2|+=1","STWV,1,<=0|ETRGCMD1|-=1"};
TriggerFunction   TrigFunc[2] = {NA_TF,NA_TF};
TriggerMode       TrigMode[2] = {NA_MODE,NA_MODE};
bool              TrigCommandString = false;

PulseCounter      pulseCounter = {0,0,false,false,false};

MFTdata Rev_1_mftdata = {
                            sizeof(MFTdata),"MFT",1,
                            115200,
                            0x1F,
                            // DAC channels
                            TW1ctrlCH,1210.19,152.87,
                            TW2ctrlCH,1217.95,25.64,
                            GRDctrlCH,1190.48,-357.14,
                            // ADC channels
                            TW1monCH,54.65,1.75,
                            TW2monCH,55.00,0,
                            GRDmonCH,55.08,-0.86,
                            0,
                            10000,
                            0,
                            true,
                            true,true,
                            false,false,
                            0xFF,0xFF,
                            {0,0},
                            {0,0},
                            0x0F,0x0F,
                            {0x0F,0x87,0xC3,0xE1,0xF0,0x78,0x3C,0x1E,0x0F,0x87,0xC3,0xE1,0xF0,0x78,0x3C,0x1E},
                            0,0,
                            0,0,
                            0,
                            // Limits
                            MINvoltage,MINvoltage,MAXvoltage,MAXvoltage,
                            MINvoltage,MAXvoltage,
                            MINfrequency,MAXfrequency,
                            //
                            SIGNATURE
                            };

char Status[20] = "Running";
int TWindx  = 0;
int TWcycl  = 0;
int TWcycls = 10;
float TW1readback = 0;
float TW2readback = 0;
float GRDreadback = 0;
extern void (*mySysTickHook)(void);
void msTimerIntercept(void);
void (*mySysTickHook)(void) = msTimerIntercept;
void msTimerIntercept(void)
{

}

// This function uses the bit pattern to fill the Twave vector.
// The flag fwd is true for forward direction
void defineTWvector(int ch, bool fwd)
{
  int mft;
  int ps;

  mft = mftdata.bitPattern[ch];
  if(fwd) ps = (mftdata.fwdPS[ch] / 45) & 0x07;
  else ps = (mftdata.revPS[ch] / 45) & 0x07;
  for(int i=0;i<8;i++)
  {
    mftdata.twave[ch][(i + ps) & 0x07] = mft & 0xFF;
    if(!fwd)
    {
      // rotate left. 8 bits
      mft = mft << 1;
      if((mft & 0x100) != 0) mft |= 1;
    }
    else
    {
      // rotate right. 8 bits
      if((mft & 1) != 0) mft |= 0x100;
      mft = (mft >> 1);
      mft &= 0xFF;
    }
  }
}

void Timer1ISR(void)
{
  static int TW1,TW2;

  TW1 = mftdata.twave[0][TWindx] | (((~mftdata.twave[0][TWindx]) & 0xFF) << 8);
  TW2 = mftdata.twave[1][TWindx] | (((~mftdata.twave[1][TWindx]) & 0xFF) << 8);
  if(mftdata.Open[0]) TW1 &= ~(mftdata.openMask[0] | (mftdata.openMask[0] << 8));
  if(mftdata.Open[1]) TW2 &= ~(mftdata.openMask[1] | (mftdata.openMask[1] << 8));
  MAX14802(TW2,TW1);
  TWindx++;
  TWindx &= 0x07;
}

void rtClockCyclsISR(void)
{
  Timer1ISR();
  if(++TWcycl > TWcycls) 
  {
    Timer1.stop();
    strcpy(Status,"Stopped");
  }
}

void setup() 
{    
  pinMode(0,OUTPUT);
  pinMode(TrigOut,OUTPUT);
  digitalWrite(TrigOut, LOW);
  pinMode(Trig1,INPUT);
  pinMode(Trig2,INPUT);
  // Read the flash config contents and test the signature
  mftdata = Rev_1_mftdata;
  Restore();
  // Init serial communications
  SerialInit();
  Serial1.begin(mftdata.Baud);
  analogReadResolution(12);
  analogWriteResolution(12);
  // Configure Threads
  SystemThread.setName((char *)"Update");
  SystemThread.onRun(Update);
  SystemThread.setInterval(25);
  // Add thread to the controller
  control.add(&SystemThread);
  // Define the vector based on bit pattern
  defineTWvector(0,mftdata.Fwd);
  defineTWvector(1,mftdata.Fwd);
  // Init the TwaveSwitch, open all switches
  MAX14802(0,0);
  // Start the clock
  mftdata.Afreq = mftdata.Freq * 8;
  // The argument sets the timer period in uS
  // This is a 16 bit timer
  int p_uS = 1000000/(mftdata.Freq * 8);
  Timer1.initialize(p_uS);
  mftdata.Afreq = 1000000/(p_uS * 8);
  Timer1.start();
  Timer1.attachInterrupt(Timer1ISR);
  // Init the TWI interface
  Wire.begin();
  // Init the DAC
  MAX5815(mftdata.MAX5815add, mftdata.TW1ctrl.Chan, Value2Counts(mftdata.TWvoltage[0],&mftdata.TW1ctrl));
  MAX5815(mftdata.MAX5815add, mftdata.TW2ctrl.Chan, Value2Counts(mftdata.TWvoltage[1],&mftdata.TW2ctrl));
  MAX5815(mftdata.MAX5815add, mftdata.GRDctrl.Chan, Value2Counts(mftdata.Guard,&mftdata.GRDctrl));
  MAX5815(mftdata.MAX5815add,3,0);
}

// This function is called at 40 Hz
void Update(void)
{
  float val;
  
  // Read the readback ADC values, filter and update global variables
  val = ReadADCchannel(mftdata.TW1mon,20);
  TW1readback = (1.0 - FILTER) * TW1readback + FILTER * val;

  val = ReadADCchannel(mftdata.TW2mon,20);
  TW2readback = (1.0 - FILTER) * TW2readback + FILTER * val;

  val = ReadADCchannel(mftdata.GRDmon,20);
  GRDreadback = (1.0 - FILTER) * GRDreadback + FILTER * val;
}

void ReadAllSerial(void)
{
  // Put serial received characters in the input ring buffer
  if (Serial1.available() > 0)
  {
    serial = &Serial1;
    PutCh(Serial1.read());
  }
  // Put serial received characters in the input ring buffer
  if (Serial.available() > 0)
  {
    serial = &Serial;
    PutCh(Serial.read());
  }
}

// This function process all the serial IO and commands
void ProcessSerial(bool scan)
{
  // Put serial received characters in the input ring buffer
  if (Serial1.available() > 0)
  {
    serial = &Serial1;
    PutCh(Serial1.read());
  }
  // Put serial received characters in the input ring buffer
  if (Serial.available() > 0)
  {
    serial = &Serial;
    PutCh(Serial.read());
  }
  if (!scan) return;
  // If there is a command in the input ring buffer, process it!
  //if (RB_Commands(&RB) > 0) while (ProcessCommand() == 0); // Process until there is nothing to do
  while (RB_Commands(&RB) > 0) ProcessCommand();
  //if((RB_Size(&RB) == 0) && TrigCommandString)
  if(TrigCommandString)
  {
    TrigCommandString = false;
    executeCommandString();
    if(SerialMute)
    {
      //while (RB_Commands(&RB) > 0) while (ProcessCommand() == 0);
      while (RB_Commands(&RB) > 0) ProcessCommand();
    }
    else
    {
      SerialMute=true;
      //while (RB_Commands(&RB) > 0) while (ProcessCommand() == 0);
      while (RB_Commands(&RB) > 0) ProcessCommand();
      SerialMute=false;
    }
  }
}

void loop() 
{
  ProcessSerial();
  control.run();
}

// Counter function

void advanceCounter(void)
{
  pulseCounter.count++;
  if(pulseCounter.count == pulseCounter.tcount)
  {
    if(pulseCounter.resetOnTcount) pulseCounter.count = 0;
    if(pulseCounter.triggerOnTcount)
    {
      if(digitalRead(TrigOut) == LOW)
      {
        digitalWrite(TrigOut, HIGH);
        delayMicroseconds(5);
        digitalWrite(TrigOut, LOW);
      }
      else
      {
        digitalWrite(TrigOut, LOW);
        delayMicroseconds(5);
        digitalWrite(TrigOut, HIGH);        
      }
    }
    if(pulseCounter.commandOnTcount)
    {
      TrigCommandString = true;
    }
  }
}

//
// Host command functions
//

int checkCH(int ch)
{
   if((ch == 1) || (ch == 2)) return ch-1;
   SetErrorCode(ERR_BADARG);
   SendNAK;
   return -1;     
}

int checkCH(char *chan)
{
   int ch = -1;

   sscanf(chan,"%d",&ch);
   if((ch == 1) || (ch == 2)) return ch-1;
   SetErrorCode(ERR_BADARG);
   SendNAK;
   return -1;     
}

bool checkTF(char *str, bool *val)
{
  if((strcmp(str,"TRUE")==0) || (strcmp(str,"FALSE")==0))
  {
    if(strcmp(str,"TRUE")==0) *val = true;
    else *val = false;
    return true;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
  return false;
}

// This function support logic if operation on select set value commands.
// The first parameter is the conditional statement, syntax
// conditional|true action|false action
// conditional options: <,>,<=,>=,=
// actions: command, user must replace commas with _ or action can be a value or change string
bool checkIF(char *str, float *change)
{
  String cond;
  bool   res;
  float  val;

  cond = str;
  cond.trim();
  if(cond.startsWith("<="))      res = *change <= cond.substring(2).toFloat();
  else if(cond.startsWith(">=")) res = *change >= cond.substring(2).toFloat();
  else if(cond.startsWith(">"))  res = *change >  cond.substring(1).toFloat();
  else if(cond.startsWith("<"))  res = *change <  cond.substring(1).toFloat();
  else if(cond.startsWith("="))  res = *change == cond.substring(1).toFloat();
  else return false;
  // Here with an if condition with the bool result in res
  // Find first |
  if(cond.indexOf('|') == -1) return true;
  cond.remove(0,cond.indexOf('|')+1);
  if(!res)
  {
    // If false find the second |
    if(cond.indexOf('|') == -1) return true;
    cond.remove(0,cond.indexOf('|')+1);
  }
  if(cond.indexOf('|') != -1) cond.remove(cond.indexOf('|'));
  // Now process the token
  if(checkChange((char *)cond.c_str(), &val)) {*change += val; return true;}
  cond.trim();
  if(isDigit(cond[0]) || (cond[0] == '.') || (cond[0] == '-'))
  {
    // If here is a value
    *change = cond.toFloat();
  }
  else
  {
    // If here its a command
    cond.replace("_",",");
    executeCommand((char *)cond.c_str());
  }
  return true;
}

bool checkIF(char *str, int *change)
{
  float val;

  val = *change;
  if(checkIF(str, &val))
  {
    *change = val;
    return true;
  }
  return false;
}

// Checks for +=value or -=value, returns true and the signed change
bool checkChange(char *str, float *change)
{
  String token;
  float  val;

  token = str;
  token.trim();
  if(token.startsWith("+=")) val = 1;
  else if(token.startsWith("-=")) val = -1;
  else return false;
  token.remove(0,2);
  *change = val * token.toFloat();
  return true;
}

bool checkChange(char *str, int *change)
{
  String token;
  int  val;

  token = str;
  token.trim();
  if(token.startsWith("+=")) val = 1;
  else if(token.startsWith("-=")) val = -1;
  else return false;
  token.remove(0,2);
  *change = val * token.toInt();
  return true;
}

bool checkPattern(char *str, int *ptrn)
{
  String token;
  int    bi  = 0;
  int    val = 0;

  token = str;
  token.trim();
  if((token.length() <= 8) && (token.length() > 0))
  {
    for(int i = token.length()-1;i>=0;i--)
    {
      if(token.charAt(i) == '1') val |= 1 << bi;
      else if(token.charAt(i) != '0')
      {
        SetErrorCode(ERR_BADCMD);
        SendNAK;
        return false;
      }
      bi++;
    }
    *ptrn = val;
    return true;
  }
  SetErrorCode(ERR_BADCMD);
  SendNAK;
  return false;
}

bool checkTrigFunc(char *value, TriggerFunction *tf)
{
  String token;

  token = value;
  token.trim();
  if(token == "REV1")          *tf = REV1_TF;
  else if(token == "REV2")     *tf = REV2_TF;
  else if(token == "OPEN1")    *tf = OPEN1_TF;
  else if(token == "OPEN2")    *tf = OPEN2_TF;
  else if(token == "CMD")      *tf = CMD_TF;
  else if(token == "CNT")      *tf = CNT_TF;
  else if(token == "TWALT1")   *tf = TWALT1_TF;
  else if(token == "TWALT2")   *tf = TWALT2_TF;
  else
  {
   SetErrorCode(ERR_BADARG);
   SendNAK;
   return false;       
  }
  return true;
}

bool checkTrigMode(char *value, TriggerMode *tm)
{
  String token;

  token = value;
  token.trim();
  if(token == "POS")         *tm = POS_MODE;
  else if(token == "NEG")    *tm = NEG_MODE;
  else if(token == "CHANGE") *tm = CHANGE_MODE;
  else if(token == "NA")     *tm = NA_MODE;
  else
  {
   *tm = NA_MODE;
   SetErrorCode(ERR_BADARG);
   SendNAK;
   return false;       
  }
  return true;
}

void SaveSettings(void)
{
  mftdata.Signature = SIGNATURE;
  EEPROM.put(eeAddress, mftdata);
  SendACK;
}

bool Restore(void)
{
  static MFTdata twsd;

  // Read the flash config contents and test the signature
  EEPROM.get(eeAddress, twsd);
  if(twsd.Signature == SIGNATURE) 
  {
    mftdata = twsd;
    return true;
  }
  return false;
}

void RestoreSettings(void)
{
  if(Restore()) 
  {
    SendACK; 
  }
  else
  {
    SetErrorCode(ERR_EEPROMWRITE);
    SendNAK;
    return;
  }
}

void Software_Reset(void)
{
  // in globals declaration section
  #define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
  #define CPU_RESTART_VAL 0x5FA0004
  #define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);


  // Then in loop do something like

  if (digitalRead(2) == HIGH)
  {
    CPU_RESTART;
  }
}

void FormatFLASH(void)
{
//  flash_twsdata.write(Rev_1_twsdata);  
  SendACK;
}

void Debug(int i)
{
}

void SetFrequency(char *value)
{
  String token;
  int    freq, p_uS;

  freq = mftdata.Freq;
  if(checkIF(value, &freq)) freq = freq;
  if(checkChange(value, &freq))
  {
    freq += mftdata.Freq;
  }
  else
  {
    token = value;
    freq = token.toInt();
  }
  if(freq > mftdata.maxFreq) freq = mftdata.maxFreq;
  if(freq < mftdata.minFreq) freq = mftdata.minFreq;
  if(freq < 1) freq = 1;
  mftdata.Freq = freq;
  p_uS = 1000000/(mftdata.Freq * 8);
  Timer1.setPeriod(p_uS);
  mftdata.Afreq = 1000000/(p_uS * 8);
  SendACK;
}

void SetFWDir(char *chan, char *fwd)
{
  int    ch;

  if((ch=checkCH(chan)) == -1) return;
  if(!checkTF(fwd, &mftdata.Fwd[ch])) return;
  defineTWvector(ch,mftdata.Fwd[ch]);
  SendACK;
}

void GetFWDir(int ch)
{
  if(checkCH(ch) == -1) return;
  SendACKonly;
  if(SerialMute) return;
  if(mftdata.Fwd[ch-1]) serial->println("TRUE");
  else serial->println("FALSE");  
}

void SetPattern(char *chan, char *ptrn)
{
  int    ch,val;

  if((ch=checkCH(chan)) == -1) return;
  if(!checkPattern(ptrn,&val)) return;
  mftdata.bitPattern[ch] = val;
  defineTWvector(ch,mftdata.Fwd[ch]);
  SendACK;
}

void GetPattern(int ch)
{
  if(checkCH(ch) == -1) return;
  SendACKonly;
  if(SerialMute) return;
  serial->println(mftdata.bitPattern[ch-1],BIN);
}

void StartTwave(void)
{
  Timer1.start();
  Timer1.attachInterrupt(Timer1ISR);
  strcpy(Status,"Running");
  SendACK;  
}

void StopTwave(void)
{
  Timer1.stop();
  strcpy(Status,"Stopped");
  SendACK;
}

void MoveNcycles(int N)
{
  TWcycl = 0;
  TWcycls = N;
  Timer1.attachInterrupt(rtClockCyclsISR);
  Timer1.start();
  strcpy(Status,"Stepping");
  SendACK;
}

void toggleTWaltV(int chan)
{
  static bool state[2] = {false,false};

  if(chan == 0)
  {
    if(state[chan]) MAX5815(mftdata.MAX5815add, mftdata.TW1ctrl.Chan, Value2Counts(mftdata.TWvoltage[0],&mftdata.TW1ctrl));
    else MAX5815(mftdata.MAX5815add, mftdata.TW1ctrl.Chan, Value2Counts(mftdata.TWaltV[0],&mftdata.TW1ctrl));
  }
  else if(chan == 1)
  {
    if(state[chan]) MAX5815(mftdata.MAX5815add, mftdata.TW2ctrl.Chan, Value2Counts(mftdata.TWvoltage[1],&mftdata.TW2ctrl));
    else MAX5815(mftdata.MAX5815add, mftdata.TW2ctrl.Chan, Value2Counts(mftdata.TWaltV[1],&mftdata.TW2ctrl));    
  }
  if(state[chan]) state[chan] = false;
  else state[chan] = true;
}

void setTWaltV(int chan, bool useALT)
{
  if(useALT)
  {
    if(chan == 0) 
      MAX5815(mftdata.MAX5815add, mftdata.TW1ctrl.Chan, Value2Counts(mftdata.TWaltV[0],&mftdata.TW1ctrl));
    if(chan == 1) 
      MAX5815(mftdata.MAX5815add, mftdata.TW2ctrl.Chan, Value2Counts(mftdata.TWaltV[1],&mftdata.TW2ctrl));
  }
  else
  {
    if(chan == 0) 
      MAX5815(mftdata.MAX5815add, mftdata.TW1ctrl.Chan, Value2Counts(mftdata.TWvoltage[0],&mftdata.TW1ctrl));
    if(chan == 1) 
      MAX5815(mftdata.MAX5815add, mftdata.TW2ctrl.Chan, Value2Counts(mftdata.TWvoltage[1],&mftdata.TW2ctrl));    
  }
}

void SetTWAvoltage(char *chan, char *value)
{
  String token;
  int    ch;
  float  val;

  if((ch=checkCH(chan)) == -1) return;
  token = value;
  val = token.toFloat();
  if((val > mftdata.maxTWV[ch]) || (val < mftdata.minTWV[ch])) BADARG;
  if(ch == 0) mftdata.TWaltV[0]=val;
  if(ch == 1) mftdata.TWaltV[1]=val;
  SendACK;  
}
void SetTWvoltage(char *chan, char *value)
{
  String token;
  int    ch;
  float  val;

  if((ch=checkCH(chan)) == -1) return;
  val = mftdata.TWvoltage[ch];
  if(checkIF(value, &val)) val = val;
  else if(checkChange(value,&val)) val += mftdata.TWvoltage[ch];
  else
  {
    token = value;
    val = token.toFloat();
  }
  if(val > mftdata.maxTWV[ch]) val = mftdata.maxTWV[ch];
  if(val < mftdata.minTWV[ch]) val = mftdata.minTWV[ch];
  if(ch == 0) 
  {
    mftdata.TWvoltage[0]=val;
    MAX5815(mftdata.MAX5815add, mftdata.TW1ctrl.Chan, Value2Counts(mftdata.TWvoltage[0],&mftdata.TW1ctrl));
  }
  if(ch == 1) 
  {
    mftdata.TWvoltage[1]=val;
    MAX5815(mftdata.MAX5815add, mftdata.TW2ctrl.Chan, Value2Counts(mftdata.TWvoltage[1],&mftdata.TW2ctrl));
  }
  SendACK;  
}

void GetTWAvoltage(int ch)
{
  if(checkCH(ch) == -1) return;
  SendACKonly;
  if(SerialMute) return;
  if(ch == 1) serial->println(mftdata.TWaltV[0]);
  if(ch == 2) serial->println(mftdata.TWaltV[1]);  
}
void GetTWvoltage(int ch)
{
  if(checkCH(ch) == -1) return;
  SendACKonly;
  if(SerialMute) return;
  if(ch == 1) serial->println(mftdata.TWvoltage[0]);
  if(ch == 2) serial->println(mftdata.TWvoltage[1]);
}

void GetTWreadback(int ch)
{
  if(checkCH(ch) == -1) return;
  SendACKonly;
  if(SerialMute) return;
  if(ch == 1) serial->println(TW1readback);
  if(ch == 2) serial->println(TW2readback);  
}

void SetGRDvoltage(char *value)
{
  String token;
  float  val;

  val = mftdata.Guard;
  if(checkIF(value, &val)) val = val;
  if(checkChange(value,&val)) val += mftdata.Guard;
  else
  {
    token = value;
    val = token.toFloat();
  }
  if(val > mftdata.maxGuard) val = mftdata.maxGuard;
  if(val < mftdata.minGuard) val = mftdata.minGuard;
  mftdata.Guard=val;
  // This code corrects non linearity at low guard voltage settings
  if(val < 3.888) val = val * 0.7553 + 0.9516;
  MAX5815(mftdata.MAX5815add, mftdata.GRDctrl.Chan, Value2Counts(val,&mftdata.GRDctrl));
  SendACK;
}

void SetGRDvoltageAdj(char *value)
{
  String token;
  float  val;

  val = mftdata.Guard;
  if(checkIF(value, &val)) val = val;
  if(checkChange(value,&val)) val += mftdata.Guard;
  else
  {
    token = value;
    val = token.toFloat();
  }
  if(val > mftdata.maxGuard) val = mftdata.maxGuard;
  if(val < mftdata.minGuard) val = mftdata.minGuard;
  mftdata.Guard=val;
  // This code corrects non linearity at low guard voltage settings, below 5 volts
  // The readback channel is used to correct
  if(val < 5.0)
  {
    float vo = val;
    for(int j=0;j<5;j++)
    {
      int cnts = Value2Counts(vo,&mftdata.GRDctrl);
      MAX5815(mftdata.MAX5815add, mftdata.GRDctrl.Chan, cnts);
      float valRB = ReadADCchannel(mftdata.GRDmon,100);
      vo += val-valRB;
    }
  }
  else MAX5815(mftdata.MAX5815add, mftdata.GRDctrl.Chan, Value2Counts(val,&mftdata.GRDctrl));
  SendACK;
}

void SetTrigOut(char *value)
{
  String token;

  token = value;
  if(token == "LOW") digitalWrite(TrigOut, LOW);
  else if(token == "HIGH") digitalWrite(TrigOut, HIGH);
  else if(token == "PULSE")
  {
    if(digitalRead(TrigOut) == LOW)
    {
      digitalWrite(TrigOut, HIGH);
      delay(1);
      digitalWrite(TrigOut, LOW);
    }
    else
    {
      digitalWrite(TrigOut, LOW);
      delay(1);
      digitalWrite(TrigOut, HIGH);
    }
  }
  else
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;    
  }
  SendACK;
}

// Calibrate TW1, TW2, and Gaurd
void Calibrate(void)
{
  String sRes;
  float  V1,V2;
  int    V1rbCnt,V2rbCnt;

// Calibrate TW1 output
   // Set votage to V1
   MAX5815(mftdata.MAX5815add,TW1ctrlCH,1000);
   // Ask user to enter V1
   V1 = UserInputFloat((char *)"\nEnter TWV1 voltage: ", NULL);
   delay(1);
   V1rbCnt = GetADCvalue(TW1monCH, 100);
   GetToken(false);  // flush the buffer
   // Set votage to V2
   MAX5815(mftdata.MAX5815add,TW1ctrlCH,20000);
   // Ask user to enter V2
   V2 = UserInputFloat((char *)"\nEnter TWV1 voltage: ", NULL);
   delay(1);
   V2rbCnt = GetADCvalue(TW1monCH, 100);
   GetToken(false);  // flush the buffer
   // Calculate the calibration parameters, m and b
   mftdata.TW1ctrl.m = 19000.0 / (V2-V1);
   mftdata.TW1ctrl.b = 20000.0 - V2 * mftdata.TW1ctrl.m;
   serial->print("\nDAC: "); serial->print(mftdata.TW1ctrl.m); serial->print(", "); serial->println(mftdata.TW1ctrl.b);
   mftdata.TW1mon.m = (V2rbCnt - V1rbCnt) / (V2-V1);
   mftdata.TW1mon.b = V2rbCnt - V2 * mftdata.TW1mon.m;
   serial->print("ADC: "); serial->print(mftdata.TW1mon.m); serial->print(", "); serial->println(mftdata.TW1mon.b);
   MAX5815(mftdata.MAX5815add, mftdata.TW1ctrl.Chan, Value2Counts(mftdata.TWvoltage[0],&mftdata.TW1ctrl));
// Calibrate TW2 output
   // Set votage to V1
   MAX5815(mftdata.MAX5815add,TW2ctrlCH,1000);
   // Ask user to enter V1
   V1 = UserInputFloat((char *)"\nEnter TWV2 voltage: ", NULL);
   delay(1);
   V1rbCnt = GetADCvalue(TW2monCH, 100);
   GetToken(false);  // flush the buffer
   // Set votage to V2
   MAX5815(mftdata.MAX5815add,TW2ctrlCH,20000);
   // Ask user to enter V2
   V2 = UserInputFloat((char *)"\nEnter TWV2 voltage: ", NULL);
   delay(1);
   V2rbCnt = GetADCvalue(TW2monCH, 100);
   GetToken(false);  // flush the buffer
   // Calculate the calibration parameters, m and b
   mftdata.TW2ctrl.m = 19000.0 / (V2-V1);
   mftdata.TW2ctrl.b = 20000.0 - V2 * mftdata.TW2ctrl.m;
   serial->print("\nDAC: "); serial->print(mftdata.TW2ctrl.m); serial->print(", "); serial->println(mftdata.TW2ctrl.b);
   mftdata.TW2mon.m = (V2rbCnt - V1rbCnt) / (V2-V1);
   mftdata.TW2mon.b = V2rbCnt - V2 * mftdata.TW2mon.m;
   serial->print("ADC: "); serial->print(mftdata.TW2mon.m); serial->print(", "); serial->println(mftdata.TW2mon.b);
   MAX5815(mftdata.MAX5815add, mftdata.TW2ctrl.Chan, Value2Counts(mftdata.TWvoltage[1],&mftdata.TW2ctrl));
// Calibrate Guard output
   // Set votage to V1
   MAX5815(mftdata.MAX5815add,GRDctrlCH,5000);
   // Ask user to enter V1
   V1 = UserInputFloat((char *)"\nEnter Guard voltage: ", NULL);
   delay(1);
   V1rbCnt = GetADCvalue(GRDmonCH, 100);
   GetToken(false);  // flush the buffer
   // Set votage to V2
   MAX5815(mftdata.MAX5815add,GRDctrlCH,20000);
   // Ask user to enter V2
   V2 = UserInputFloat((char *)"\nEnter Guard voltage: ", NULL);
   delay(1);
   V2rbCnt = GetADCvalue(GRDmonCH, 100);
   GetToken(false);  // flush the buffer
   // Calculate the calibration parameters, m and b
   mftdata.GRDctrl.m = 15000.0 / (V2-V1);
   mftdata.GRDctrl.b = 20000.0 - V2 * mftdata.GRDctrl.m;
   serial->print("\nDAC: "); serial->print(mftdata.GRDctrl.m); serial->print(", "); serial->println(mftdata.GRDctrl.b);
   mftdata.GRDmon.m = (V2rbCnt - V1rbCnt) / (V2-V1);
   mftdata.GRDmon.b = V2rbCnt - V2 * mftdata.GRDmon.m;
   serial->print("ADC: "); serial->print(mftdata.GRDmon.m); serial->print(", "); serial->println(mftdata.GRDmon.b);
   MAX5815(mftdata.MAX5815add, mftdata.GRDctrl.Chan, Value2Counts(mftdata.Guard,&mftdata.GRDctrl));
}

void setOpen(char *chan, char *val)
{
  int    ch;

  if((ch=checkCH(chan)) == -1) return;
  if(!checkTF(val, &mftdata.Open[ch])) return;
  defineTWvector(ch,mftdata.Fwd[ch]);
  SendACK;  
}
void getOpen(int ch)
{
  if(checkCH(ch) == -1) return;
  SendACKonly;
  if(SerialMute) return;
  if(mftdata.Open[ch-1]) serial->println("TRUE");
  else serial->println("FALSE");    
}
void setOpenMask(char *chan, char *ptrn)
{
  int    ch,val;

  if((ch=checkCH(chan)) == -1) return;
  if(!checkPattern(ptrn,&val)) return;
  mftdata.openMask[ch] = val;
  defineTWvector(ch,mftdata.Fwd[ch]);
  SendACK;
  
}
void getOpenMask(int ch)
{
  if(checkCH(ch) == -1) return;
  SendACKonly;
  if(SerialMute) return;
  serial->println(mftdata.openMask[ch-1],BIN);  
}
void setFwdPS(char *chan, char *phase)
{
  String token;
  int ch,ph; 

  token = phase;
  if((ch=checkCH(chan)) == -1) return;
  if(checkChange(phase, &ph)) ph += mftdata.fwdPS[ch];
  else ph = token.toInt();
  SendACK;
  mftdata.fwdPS[ch] = ph;
  defineTWvector(ch,mftdata.Fwd[ch]);
}
void getFwdPS(int ch)
{
  if(checkCH(ch) == -1) return;
  SendACKonly;
  if(SerialMute) return;
  serial->println(mftdata.fwdPS[ch-1]);  
}
void setRevPS(char *chan, char *phase)
{
  String token;
  int ch,ph; 

  token = phase;
  if((ch=checkCH(chan)) == -1) return;
  if(checkChange(phase, &ph)) ph += mftdata.revPS[ch];
  else ph = token.toInt();
  SendACK;
  mftdata.revPS[ch] = ph;
  defineTWvector(ch,mftdata.Fwd[ch]);
}
void getRevPS(int ch)
{
  if(checkCH(ch) == -1) return;
  SendACKonly;
  if(SerialMute) return;
  serial->println(mftdata.revPS[ch-1]);  
}

void putString(char *str)
{
   //for(unsigned int i=0;i<strlen(str);i++) PutCh(str[i]);  
   //return;
   for(int i=strlen(str);i>0;i--) 
   {
    PushCh(str[i-1]);  
   }
}

void executeCommand(char *str)
{
  if(!SerialMute) putString((char *)"MUTE,OFF\n");
  putString((char *)"\n");
  putString(str);
  if(!SerialMute) putString((char *)"MUTE,ON\n");
}

void executeCommandString(void)
{
  // If the command string length is greater that zero then put
  // the string in the command ring buffer for execution. 
  if(strlen(commandString[activeCS]) > 0)
  {
    putString((char *)"\n");
    putString(commandString[activeCS]);
  }
}

void playCommandString(void)
{
  if(!SerialMute) putString((char *)"MUTE,OFF\n");
  executeCommandString();
  if(!SerialMute) putString((char *)"MUTE,ON\n");
  SendACKonly;
}

void playCommandString1(void) {activeCS=0; playCommandString();}
void playCommandString2(void) {activeCS=1; playCommandString();}

void Trig1isr(void)
{
  int state;

  delayMicroseconds(10);
  state = digitalRead(Trig1);
  switch (TrigFunc[0])
  {
    case REV1_TF:
      if((TrigMode[0] == POS_MODE) && (state == HIGH)) mftdata.Fwd[0] = false;
      if((TrigMode[0] == POS_MODE) && (state == LOW))  mftdata.Fwd[0] = true;
      if((TrigMode[0] == NEG_MODE) && (state == LOW))  mftdata.Fwd[0] = true;
      if((TrigMode[0] == NEG_MODE) && (state == HIGH)) mftdata.Fwd[0] = false;
      break;
    case REV2_TF:
      if((TrigMode[0] == POS_MODE) && (state == HIGH)) mftdata.Fwd[1] = false;
      if((TrigMode[0] == POS_MODE) && (state == LOW))  mftdata.Fwd[1] = true;
      if((TrigMode[0] == NEG_MODE) && (state == LOW))  mftdata.Fwd[1] = true;
      if((TrigMode[0] == NEG_MODE) && (state == HIGH)) mftdata.Fwd[1] = false;
      break;
    case OPEN1_TF:
      if((TrigMode[0] == POS_MODE) && (state == HIGH)) mftdata.Open[0] = false;
      if((TrigMode[0] == POS_MODE) && (state == LOW))  mftdata.Open[0] = true;
      if((TrigMode[0] == NEG_MODE) && (state == LOW))  mftdata.Open[0] = true;
      if((TrigMode[0] == NEG_MODE) && (state == HIGH)) mftdata.Open[0] = false;
      break;
    case OPEN2_TF:
      if((TrigMode[0] == POS_MODE) && (state == HIGH)) mftdata.Open[1] = false;
      if((TrigMode[0] == POS_MODE) && (state == LOW))  mftdata.Open[1] = true;
      if((TrigMode[0] == NEG_MODE) && (state == LOW))  mftdata.Open[1] = true;
      if((TrigMode[0] == NEG_MODE) && (state == HIGH)) mftdata.Open[1] = false;
      break;
    case CMD_TF:
      if((TrigMode[0] == POS_MODE) && (state == HIGH)) TrigCommandString = true;
      if((TrigMode[0] == NEG_MODE) && (state == LOW))  TrigCommandString = true;
      if(TrigMode[0] == CHANGE_MODE)  TrigCommandString = true;
      break;
    case CNT_TF:
      if((TrigMode[0] == POS_MODE) && (state == HIGH)) advanceCounter();
      if((TrigMode[0] == NEG_MODE) && (state == LOW))  advanceCounter();
      if(TrigMode[0] == CHANGE_MODE)  advanceCounter();
      break;
    case TWALT1_TF:
      if((TrigMode[0] == POS_MODE) && (state == HIGH)) setTWaltV(0,true);
      if((TrigMode[0] == POS_MODE) && (state == LOW))  setTWaltV(0,false);
      if((TrigMode[0] == NEG_MODE) && (state == LOW))  setTWaltV(0,true);
      if((TrigMode[0] == NEG_MODE) && (state == HIGH)) setTWaltV(0,false);
      if(TrigMode[0] == CHANGE_MODE)  toggleTWaltV(0);
      break;
    case TWALT2_TF:
      if((TrigMode[0] == POS_MODE) && (state == HIGH)) setTWaltV(1,true);
      if((TrigMode[0] == POS_MODE) && (state == LOW))  setTWaltV(1,false);
      if((TrigMode[0] == NEG_MODE) && (state == LOW))  setTWaltV(1,true);
      if((TrigMode[0] == NEG_MODE) && (state == HIGH)) setTWaltV(1,false);
      if(TrigMode[0] == CHANGE_MODE)  toggleTWaltV(0);
      break;
    default:
      break;
  }
}

// This function enables the trigger 1 input and assigns it a function.
// mode defines the trigger level, NA,POS,NEG,CHANGE. NA disables
// function defines the trigger action, REV 1or2, OPEN 1or2, CMD, CNT
void setTrig1(char *mode, char *function)
{
  if(!checkTrigFunc(function, &TrigFunc[0])) return;
  if(!checkTrigMode(mode, &TrigMode[0])) return;
  if(TrigMode[0] == POS_MODE)         attachInterrupt(digitalPinToInterrupt(Trig1), Trig1isr, CHANGE);
  else if(TrigMode[0] == NEG_MODE)    attachInterrupt(digitalPinToInterrupt(Trig1), Trig1isr, CHANGE);
  else if(TrigMode[0] == CHANGE_MODE) attachInterrupt(digitalPinToInterrupt(Trig1), Trig1isr, CHANGE);
  else if(TrigMode[0] == NA_MODE)     detachInterrupt(digitalPinToInterrupt(Trig1));
  SendACK;
}

void Trig2isr(void)
{
  int state;

  delayMicroseconds(10);
  state = digitalRead(Trig2);
  switch (TrigFunc[1])
  {
    case REV1_TF:
      if((TrigMode[1] == POS_MODE) && (state == HIGH)) mftdata.Fwd[0] = false;
      if((TrigMode[1] == POS_MODE) && (state == LOW))  mftdata.Fwd[0] = true;
      if((TrigMode[1] == NEG_MODE) && (state == LOW))  mftdata.Fwd[0] = true;
      if((TrigMode[1] == NEG_MODE) && (state == HIGH)) mftdata.Fwd[0] = false;
      break;
    case REV2_TF:
      if((TrigMode[1] == POS_MODE) && (state == HIGH)) mftdata.Fwd[1] = false;
      if((TrigMode[1] == POS_MODE) && (state == LOW))  mftdata.Fwd[1] = true;
      if((TrigMode[1] == NEG_MODE) && (state == LOW))  mftdata.Fwd[1] = true;
      if((TrigMode[1] == NEG_MODE) && (state == HIGH)) mftdata.Fwd[1] = false;
      break;
    case OPEN1_TF:
      if((TrigMode[1] == POS_MODE) && (state == HIGH)) mftdata.Open[0] = false;
      if((TrigMode[1] == POS_MODE) && (state == LOW))  mftdata.Open[0] = true;
      if((TrigMode[1] == NEG_MODE) && (state == LOW))  mftdata.Open[0] = true;
      if((TrigMode[1] == NEG_MODE) && (state == HIGH)) mftdata.Open[0] = false;
      break;
    case OPEN2_TF:
      if((TrigMode[1] == POS_MODE) && (state == HIGH)) mftdata.Open[1] = false;
      if((TrigMode[1] == POS_MODE) && (state == LOW))  mftdata.Open[1] = true;
      if((TrigMode[1] == NEG_MODE) && (state == LOW))  mftdata.Open[1] = true;
      if((TrigMode[1] == NEG_MODE) && (state == HIGH)) mftdata.Open[1] = false;
      break;
    case CMD_TF:
      if((TrigMode[1] == POS_MODE) && (state == HIGH)) TrigCommandString = true;
      if((TrigMode[1] == NEG_MODE) && (state == LOW))  TrigCommandString = true;
      if(TrigMode[1] == CHANGE_MODE)  TrigCommandString = true;
      break;
    case CNT_TF:
      if((TrigMode[1] == POS_MODE) && (state == HIGH)) advanceCounter();
      if((TrigMode[1] == NEG_MODE) && (state == LOW))  advanceCounter();
      if(TrigMode[1] == CHANGE_MODE)  advanceCounter();
      break;
    case TWALT1_TF:
      if((TrigMode[1] == POS_MODE) && (state == HIGH)) setTWaltV(0,true);
      if((TrigMode[1] == POS_MODE) && (state == LOW))  setTWaltV(0,false);
      if((TrigMode[1] == NEG_MODE) && (state == LOW))  setTWaltV(0,true);
      if((TrigMode[1] == NEG_MODE) && (state == HIGH)) setTWaltV(0,false);
      if(TrigMode[1] == CHANGE_MODE)  toggleTWaltV(0);
      break;
    case TWALT2_TF:
      if((TrigMode[1] == POS_MODE) && (state == HIGH)) setTWaltV(1,true);
      if((TrigMode[1] == POS_MODE) && (state == LOW))  setTWaltV(1,false);
      if((TrigMode[1] == NEG_MODE) && (state == LOW))  setTWaltV(1,true);
      if((TrigMode[1] == NEG_MODE) && (state == HIGH)) setTWaltV(1,false);
      if(TrigMode[1] == CHANGE_MODE)  toggleTWaltV(1);
      break;
    default:
      break;
  }
}

// This function enables the trigger 2 input and assigns it a function.
// mode defines the trigger level, NA,POS,NEG,CHANGE. NA disables
// function defines the trigger action, REV 1or2, OPEN 1or2, CMD, CNT
void setTrig2(char *mode, char *function)
{
  if(!checkTrigFunc(function, &TrigFunc[1])) return;
  if(!checkTrigMode(mode, &TrigMode[1])) return;
  if(TrigMode[1] == POS_MODE)         attachInterrupt(digitalPinToInterrupt(Trig2), Trig2isr, CHANGE);
  else if(TrigMode[1] == NEG_MODE)    attachInterrupt(digitalPinToInterrupt(Trig2), Trig2isr, CHANGE);
  else if(TrigMode[1] == CHANGE_MODE) attachInterrupt(digitalPinToInterrupt(Trig2), Trig2isr, CHANGE);
  else if(TrigMode[1] == NA_MODE)     detachInterrupt(digitalPinToInterrupt(Trig2));
  SendACK;
}

void clearCounter(void)
{
  pulseCounter.count = 0;
  SendACK;
}

void readTriggerInput(int ch)
{
  int i;
  
  if(ch == 1) i = digitalRead(Trig1);
  else if(ch == 2) i = digitalRead(Trig2);
  else BADARG;
  SendACKonly;
  if(SerialMute) return;
  serial->println(i);
}

// User limit functions

// Called with parameters in the ring buffer
void setLimit(bool MAXvalue)
{
  char        *tkn;
  String      arg,type;
  int         chan=1;
  float       limit;

  while(true)
  {
     if((tkn = TokenFromCommandLine(',')) == NULL) break;
     type = tkn;
     if(type == "TWV")
     {
        // Read the channel
        if((tkn = TokenFromCommandLine(',')) == NULL) break;
        arg = tkn;
        chan = arg.toInt();
     }
     if((tkn = TokenFromCommandLine(',')) == NULL) break;
     arg = tkn;
     limit = arg.toFloat();
     if(checkCH(chan) == -1) return;
     chan--;
     if(type == "TWV")
     {
        if((limit < MINvoltage) || (limit > MAXvoltage)) break;
        if(MAXvalue) mftdata.maxTWV[chan] = limit;
        else mftdata.minTWV[chan] = limit;
     }
     else if(type == "GRD")
     {
        if((limit < MINvoltage) || (limit > MAXvoltage)) break;
        if(MAXvalue) mftdata.maxGuard = limit;
        else mftdata.minGuard = limit;
     }
     else if(type == "FREQ")
     {
        if((limit < MINfrequency) || (limit > MAXfrequency)) break;
        if(MAXvalue) mftdata.maxFreq = limit;
        else mftdata.minFreq = limit;    
     }
     else break;
     SendACK;
     return;
  }
  BADARG;
}

// Called with parameters in the ring buffer
void getLimit(bool MAXvalue)
{
  char        *tkn;
  String      arg,type;
  int         chan=1;

  while(true)
  {
     if((tkn = TokenFromCommandLine(',')) == NULL) break;
     type = tkn;
     if(type == "TWV")
     {
        // Read the channel
        if((tkn = TokenFromCommandLine(',')) == NULL) break;
        arg = tkn;
        chan = arg.toInt();
     }
     if(checkCH(chan) == -1) return;
     if(type == "TWV")
     {
        if(!SerialMute)
        {
          SendACKonly;
          if(MAXvalue) serial->println(mftdata.maxTWV[chan]);
          else serial->println(mftdata.minTWV[chan]);
        }
     }
     else if(type == "GRD")
     {
        if(!SerialMute)
        {
          SendACKonly;
          if(MAXvalue) serial->println(mftdata.maxGuard);
          else serial->println(mftdata.minGuard);
        }
     }
     else if(type == "FREQ")
     {
        if(!SerialMute)
        {
          SendACKonly;
          if(MAXvalue) serial->println(mftdata.maxFreq);
          else serial->println(mftdata.minFreq);
        }
     }
     else break;
     return;    
  }
  BADARG;
}

void setMaximum(void) { setLimit(true); }
void getMaximum(void) { setLimit(true); }
void setMinimum(void) { setLimit(false); }
void getMinimum(void) { setLimit(false); }

void setActiveCMD(int ch)
{
  if(checkCH(ch) == -1) return;
  activeCS = ch - 1;
  SendACK;
}

void getActiveCMD(void)
{
  SendACKonly;
  if(!SerialMute) serial->println(activeCS+1);
}

// Clock functions

void clockCounter(bool high)
{
  if(high) advanceCounter();
}
void clockTrigger(bool high)
{
  if(high) digitalWrite(TrigOut, HIGH);
  else digitalWrite(TrigOut, LOW);
}
void clockISR(void)
{
  static bool high = false;

  if(high) high = false;
  else high = true;
  if(clockFunction != NULL) clockFunction(high);
}

void setClock(int freq)
{
  if((freq < 0) || (freq > 10000)) BADARG;
  clockFrequency = freq;
  if(freq == 0) clockTimer.end();
  clockTimer.begin(clockISR, 1000000/(2 * freq));
  SendACK;
}

void setClockFunction(char *func)
{
  String token;

  token = func;
  if(token == "NA")       clockFunction = NULL;
  else if(token == "CNT") clockFunction = clockCounter;
  else if(token == "TRG") clockFunction = clockTrigger;
  else BADARG;
  strcpy(clockMode,token.c_str());
  SendACK;
}
