/*
 * in kinetis.h
 * define F_BUS 80000
 * 
 * CPU=240MHz; BUS=80MHz => SYSCLK=13.3MHz max
 * CPU=216MHz; BUS=72MHz => SYSCLK=12MHz max
 * CPU=192MHz; BUS=64MHz => SYSCLK=10.7MHz max
 * CPU=180MHz; BUS=60MHz => SYSCLK=10MHz max
 */

#include <i2c_t3.h>
#include "mt9v034.h"


#define STANDBY     0
#define SYSCLK      3
#define EXPOSURE   29
#define STFRM_OUT  30
#define STLN_OUT1  31
#define STLN_OUT2  32
#define LNVAL1     25
#define FRVAL1     26
#define PIXCLK1    27
#define LNVAL2     40
#define FRVAL2     41
#define PIXCLK2    42

#define PORT_CAM1  GPIOC_PDIR
#define PORT_CAM2  GPIOD_PDIR

mt9v034 cam1(&Wire, 0x90>>1);
mt9v034 cam2(&Wire, 0xB0>>1);

// pin 13 is part of PORTC!
//int led = 13;

#define N_PIXELS  751

uint16_t exposure_us = 10000;   // up to 16383 us
uint8_t analogGain = 16;  // 16 - 64
uint8_t digitalGain = 4;  // 0 - 15
uint16_t n_lines = MAX_IMAGE_HEIGHT;
bool send_picture_data = false;

void setup()
{
  // configure data pins for INPUT
  // (this should eventually be replaced with more clever direct port manupulation)
  ConfigDataPins();
  
  // setup SYSCLK to 13.3MHz
  pinMode(SYSCLK, OUTPUT);
  analogWriteResolution(2); // increasing resolution doesn't help
  analogWriteFrequency(SYSCLK, 13300000); // <= BUS / n
  analogWrite(SYSCLK, 2);   // about 50% duty cycle

  pinMode(STANDBY, OUTPUT);
  digitalWrite(STANDBY, LOW);
  
  // Setup I2C for Master mode, pins 18/19, external pullups, 100kHz, 200ms default timeout
  Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, 100000);
  Wire.setDefaultTimeout(200000); // 200ms

  // configure camera for Slave mode
  cam1.initialize();
  cam2.initialize();

  cam1.setAGC(false);
  cam2.setAGC(false);
  cam1.setCompanding(true);
  cam2.setCompanding(true);

  pinMode(LNVAL1, INPUT);
  pinMode(FRVAL1, INPUT);
  pinMode(PIXCLK1, INPUT);
  pinMode(LNVAL2, INPUT);
  pinMode(FRVAL2, INPUT);
  pinMode(PIXCLK2, INPUT);

  pinMode(EXPOSURE, OUTPUT);
  pinMode(STFRM_OUT, OUTPUT);
  pinMode(STLN_OUT1, OUTPUT);

  Serial.begin(9600);  // USB FS is always 12 Mbit/sec
  
  delay(5000);
}


void loop() 
{
  uint16_t ln_cnt = 0;
  uint16_t nv_cnt = 0;
  uint16_t zr_cnt = 0;

  uint16_t a[MAX_IMAGE_WIDTH], b[MAX_IMAGE_WIDTH];

  static bool second_frame = 0;

  PulsePin(EXPOSURE, 50);
  delayMicroseconds(exposure_us);
  PulsePin(STFRM_OUT, 50);

  for(int ln=0; ln<530; ln++)   // min 525 lines, including blanking
  {
    PulsePin(STLN_OUT1, 50);
    if( ReadCameraLine(a, b, MAX_IMAGE_WIDTH) )
    {
      // make sure last data point contains valid line and frame markers
      if( (a[0]>>10) != 0x3 || (b[0]>>10) != 0xC )
        zr_cnt++;
      else
        if( (ln_cnt >= MAX_IMAGE_HEIGHT/2 - n_lines/2) &&
            (ln_cnt <  MAX_IMAGE_HEIGHT/2 + n_lines/2) && send_picture_data && second_frame )
        { // send data out on every second frame (seems to improve noise)
          SendLine(0, ln_cnt, a, MAX_IMAGE_WIDTH);
          SendLine(1, ln_cnt, b, MAX_IMAGE_WIDTH);
        }

      ln_cnt++;
    }
    else
      nv_cnt++;
  }

  second_frame = ! second_frame;

  while( Serial.available() )
  {
    uint8_t buf[3];
    //Serial.readBytes((char*)buf, 3);  // doesn't work!
    for(int i=0; i<3; i++)
      buf[i] = Serial.read();
    
    switch( buf[0] )
    {
      case 'E' :  exposure_us = buf[1] << 8;
                  exposure_us |= buf[2];
                  break;
      case 'A' :  analogGain = buf[2];
                  cam1.setAnalogGain(analogGain);
                  cam2.setAnalogGain(analogGain);
                  break;
      case 'D' :  digitalGain = buf[2];
                  cam1.setDigitalGain(digitalGain);
                  cam2.setDigitalGain(digitalGain);
                  break;
      case 'N' :  n_lines = buf[1] << 8;
                  n_lines |= buf[2];
                  break;
      case 'P' :  send_picture_data = buf[2] & 0x01;
                  break;
      default  :  break;
    }
  }

  if( ! Serial.dtr() )
    send_picture_data = false;
}


void SendLine(bool cam, uint16_t line, uint16_t *data, uint16_t dataSize)
{
  uint8_t *packed = new uint8_t[dataSize+2];

  Pack8bits(packed+2, data, dataSize);
  packed[0] = (cam ? (1<<7) : 0) | (line >> 8);
  packed[1] = line;
  Serial.write(packed, dataSize+2);
  Serial.send_now();

  delete[] packed;
}


//
// dstLength = srcLength
//
void Pack8bits(uint8_t *dstByte, uint16_t *srcWord, size_t srcLength)
{
  for (size_t i = 0; i < srcLength; i++)
  {
    dstByte[i] = srcWord[i] >> 2;
  }
}


//
// dstLength = srcLength / 4 * 5
//
void Pack10bits(uint8_t *dstByte, uint16_t *srcWord, size_t srcLength)
{
  for (size_t i = 0; i < srcLength; i+=4)
  {
    dstByte[0] = srcWord[0];
    dstByte[1] = ((srcWord[0] & 0x3FF) >> 8) | (srcWord[1] << 2);
    dstByte[2] = ((srcWord[1] & 0x3FF) >> 6) | (srcWord[2] << 4);
    dstByte[3] = ((srcWord[2] & 0x3FF) >> 4) | (srcWord[3] << 6);
    dstByte[4] = ((srcWord[3] & 0x3FF) >> 2);
    dstByte += 5;
    srcWord += 4;
  }
}


void PulsePin(int pin, uint32_t cnt)
{
   digitalWriteFast(pin, 1);
   CountPixclks(cnt);
   digitalWriteFast(pin, 0);
}


void CountPixclks(uint32_t cnt)
{
  while( cnt != 0 )
  {
//    asm volatile("mov r0, r0");
    if ( digitalReadFast(PIXCLK1) )
      cnt--;
  }
}


bool ReadCameraLine(uint16_t *a, uint16_t *b, uint16_t px)
{
//  int px = 750;  // max (752-1)
  px -= 2;

  noInterrupts();
  
  // PIXCLKs between STLN_OUT1 pulses
  // need about 60 extra pulses before LINE_VALID, so min is 752 + 60 = 812
  int cnt = 820;
  // if no LINE_VALID, simply count PIXCLKs until it's time for next STLN_OUT pulse
  while( ! digitalReadFast(LNVAL1) )
  { // count PIXCLKs
    if ( digitalReadFast(PIXCLK1) )
    {
      if (--cnt == 0)
      {
        interrupts();
        return false;
      }
    }
  }

  // capture one line of data
  while(1)
  {
    // sample data on falling edge of PIXCLK (first pixel may be lost)
    if ( ! digitalReadFast(PIXCLK1) )
    {
      uint16_t d1 = PORT_CAM1;
      uint16_t d2 = PORT_CAM2;
      a[px] = d1;
      b[px] = d2;
      if( px-- == 0 )
        break;
    }
  }

  interrupts();
  return true;
}


void ConfigDataPins(void)
{
  pinMode(2, INPUT);  // D0
  pinMode(5, INPUT);  // D7
  pinMode(6, INPUT);  // D4
  pinMode(7, INPUT);  // D2
  pinMode(8, INPUT);  // D3
  pinMode(9, INPUT);  // C3
  pinMode(10, INPUT);  // C4
  pinMode(11, INPUT);  // C6
  pinMode(12, INPUT);  // C7
  pinMode(13, INPUT);  // C5
  pinMode(14, INPUT);  // D1
  pinMode(15, INPUT);  // C0
  pinMode(20, INPUT);  // D5
  pinMode(21, INPUT);  // D6
  pinMode(22, INPUT);  // C1
  pinMode(23, INPUT);  // C2
  pinMode(35, INPUT);  // C8
  pinMode(36, INPUT);  // C9
  pinMode(47, INPUT);  // D8
  pinMode(48, INPUT);  // D9

  // extra debug pins
  pinMode(37, INPUT);  // C10
  pinMode(38, INPUT);  // C11
  pinMode(52, INPUT);  // D13
  pinMode(53, INPUT);  // D12
}

