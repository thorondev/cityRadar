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
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void processInputs(AudioSystem::Config& config, bool& sendOutput)
{
  // control input from serial
  while(Serial.available())
  {
    int8_t input = Serial.read();

    if(input==100) // 'd' for data
        sendOutput=true;

    if((input==0) & (config.mic_gain > 0.001)){ // - key
        config.mic_gain -= 0.01;
    }
    if((input==1) & (config.mic_gain<10)){ // + key
        config.mic_gain += 0.01;
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
        Serial.print("alpha = ");
        Serial.println(config.alpha);
        Serial.print("psi = ");
        Serial.println(config.psi);

        config.hasChanges = true;
    }

    if(input==84)
    {
        unsigned long externalTime = Serial.parseInt();

        // check if we got a somewhat recent time (greater than March 20 2024)
        unsigned long const minimalTime = 1710930219;
        if( externalTime >= minimalTime)
        {
            setTime(externalTime);          // Sync Arduino clock to the time received on the serial port
            Teensy3Clock.set(externalTime); // set Teensy RTC
        }

        Serial.print("Time set to: ");
        Serial.print(year());
        Serial.print("-");
        Serial.print(month());
        Serial.print("-");
        Serial.print(day());
        Serial.print(" ");
        Serial.print(hour());
        printDigits(minute());
        printDigits(second());
        Serial.println();
    }
  }
}
