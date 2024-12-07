#include "Hardware.h"
#include "AtomicBlock.h"
#include <Arduino.h>
#include "SPI.h"
#include <wiring_private.h>
#include <assert.h>

// Reads the selected ADC channel for the number of averages defined by num
int GetADCvalue(int chan, int num)
{
  int i=0,j;

  for(j=0;j<num;j++) i += analogRead(chan);
  return i/num;
}

// This function reads and returns an ADC channel value. The raw ADC
// value is read and converted to engineering units.
float ReadADCchannel(ADCchan adch, int num)
{
  int adc = GetADCvalue(adch.Chan,num);
  return Counts2Value(adc,&adch);
}

// Counts to value and value to count conversion functions.
// Overloaded for both DACchan and ADCchan structs.
float Counts2Value(int Counts, DACchan *DC)
{
  return (Counts - DC->b) / DC->m;
}

float Counts2Value(int Counts, ADCchan *ad)
{
  return (Counts - ad->b) / ad->m;
}

int Value2Counts(float Value, DACchan *DC)
{
  int counts;

  counts = (Value * DC->m) + DC->b;
  if (counts < 0) counts = 0;
  if (counts > 65535) counts = 65535;
  return (counts);
}

int Value2Counts(float Value, ADCchan *ac)
{
  int counts;

  counts = (Value * ac->m) + ac->b;
  if (counts < 0) counts = 0;
  if (counts > 65535) counts = 65535;
  return (counts);
}

void ComputeCRCbyte(byte *crc, byte by)
{
  byte generator = 0x1D;

  *crc ^= by;
  for(int j=0; j<8; j++)
  {
    if((*crc & 0x80) != 0)
    {
      *crc = ((*crc << 1) ^ generator);
    }
    else
    {
      *crc <<= 1;
    }
  }
}

// Compute 8 bit CRC of buffer
byte ComputeCRC(byte *buf, int bsize)
{
  byte generator = 0x1D;
  byte crc = 0;

  for(int i=0; i<bsize; i++)
  {
    crc ^= buf[i];
    for(int j=0; j<8; j++)
    {
      if((crc & 0x80) != 0)
      {
        crc = ((crc << 1) ^ generator);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void MAX14802_Latch(void)
{
  digitalWrite(LTCH, LOW);
  digitalWrite(LTCH, HIGH);
}

SPISettings settingsA(20000000, MSBFIRST, SPI_MODE2);
void MAX14802(int TW1, int TW2, bool Latch)
{
  static bool inited = false;

  if(!inited)
  {
    // Init the SPI interface
    SPI.begin();
    SPI.beginTransaction(settingsA);
    // Set latch high
    pinMode(LTCH, OUTPUT);
    digitalWrite(LTCH, HIGH);
    // Set clear low
    pinMode(CLRMUX, OUTPUT);
    digitalWrite(CLRMUX, LOW);
    inited = true;
  }
  SPI.transfer16(TW1);
  SPI.transfer16(TW2);
  if(Latch) MAX14802_Latch();
}

void MAX5815(int addr, int chn, int counts)
{
  static bool inited = false;

  if(!inited)
  {
    pinMode(CLRDAC,OUTPUT);
    pinMode(LDAC,OUTPUT);
    digitalWrite(CLRDAC,LOW);
    delay(1);
    digitalWrite(CLRDAC,HIGH);
    digitalWrite(LDAC,LOW);
    // Turn on internal reference
    Wire.beginTransmission(addr);
    Wire.write(0x75);
    Wire.write(0x0);
    Wire.write(0x0);
    Wire.endTransmission();
    delay(10);
    inited = true;
  }
  Wire.beginTransmission(addr);
  Wire.write(0x30 | chn);
  Wire.write(counts >> 8);
  Wire.write(counts);
  Wire.endTransmission();
}
