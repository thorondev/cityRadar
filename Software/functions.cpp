#include "functions.h"

#include <Audio.h>
#include <OpenAudio_ArduinoLibrary.h>
#include <AudioStream_F32.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <utility/imxrt_hw.h>
#include <TimeLib.h>

void setI2SFreq(int freq) {
  // PLL between 27*24 = 648MHz und 54*24=1296MHz
  int n1 = 4; //SAI prescaler 4 => (n1*n2) = multiple of 4
  int n2 = 1 + (24000000 * 27) / (freq * 256 * n1);
  double C = ((double)freq * 256 * n1 * n2) / 24000000;
  int c0 = C;
  int c2 = 10000;
  int c1 = C * c2 - (c0 * c2);
  set_audioClock(c0, c1, c2, true);
  CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK))
       | CCM_CS1CDR_SAI1_CLK_PRED(n1-1) // &0x07
       | CCM_CS1CDR_SAI1_CLK_PODF(n2-1); // &0x3f
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  // TODO changed from SerialUSB1
  SerialUSB.print(":");
  if(digits < 10)
    SerialUSB.print('0');
  SerialUSB.print(digits);
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void processInputs(AudioSystem::Config& config)
{
  // control input from serial
  if (Serial.available() > 0) {
    int input = Serial.read();

    if((input==0) & (config.mic_gain > 0.001)){ // - key
        config.mic_gain -= 0.01;
    }
    if((input==1) & (config.mic_gain<10)){ // + key
        config.mic_gain += 0.01;
    }

    if(input==100){ // d for data
        send_output=true;
    }

    if(input==111){ //o
        config.alpha += 0.01;
    }
    if(input==108){ //l
        config.alpha -= 0.01;
    }
    if(input==105){ //i
        config.psi += 0.01;
    }
    if(input==107){ //k
        config.psi -= 0.01;
    }

    if(input==111 || input==108 || input==105 || input==107){
        SerialUSB1.print("alpha = ");
        SerialUSB1.println(config.alpha);
        SerialUSB1.print("psi = ");
        SerialUSB1.println(config.psi);

        config.hasChanges = true;
    }

    if(input==84)
    {
        unsigned long pctime = Serial.parseInt();

        // check the integer is a valid time (greater than Jan 1 2013)
        if( pctime >= DEFAULT_TIME)
        {
            setTime(pctime); // Sync Arduino clock to the time received on the serial port
            Teensy3Clock.set(pctime); // set Teensy RTC
        }

        SerialUSB1.print("Time set to: ");
        SerialUSB1.print(year());
        SerialUSB1.print("-");
        SerialUSB1.print(month());
        SerialUSB1.print("-");
        SerialUSB1.print(day());
        SerialUSB1.print(" ");
        SerialUSB1.print(hour());
        printDigits(minute());
        printDigits(second());
        SerialUSB1.println();
    }
  }
}
