#include "AudioSystem.h"
#include "Config.h"
#include "functions.h"
#include "noise_floor.h"

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <utility/imxrt_hw.h>
#include <TimeLib.h>

// Audio shield SD card:
#define SDCARD_CS_PIN    10
#define SDCARD_MOSI_PIN  11   // Teensy 4 ignores this, uses pin 11
#define SDCARD_SCK_PIN   13  // Teensy 4 ignores this, uses pin 13

// builtin SD card:
// #define SDCARD_CS_PIN    BUILTIN_SDCARD
// #define SDCARD_MOSI_PIN  11  // not actually used
// #define SDCARD_SCK_PIN   13  // not actually used

//variables for file stuff
File data_file;               // file for raw data
File csv_file;                // file for metrics as csv
time_t timestamp;             // time stamp for saved data
bool send_output = false;     // send output over serial? (ONCE)
uint16_t file_version = 1.1;  // file version of the output
const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 // TODO rename


AudioSystem audio;
AudioSystem::Results audioResults; // global to optimize for speed
Config config;

void setup() {
  setSyncProvider(getTeensy3Time);
  setI2SFreq(config.audio.sample_rate);

  // TODO maybe move to audio
  pinMode(PIN_A3, OUTPUT); //A3=17, A8=22, A4=18
  digitalWrite(PIN_A4, LOW);

  audio.setup(config.audio, config.max_pedestrian_speed, config.send_max_speed);

  // TODO
  // sdCard.setup();

  //============== SD card =============

  // Configure SPI
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);

  //setTime(Teensy3Clock.get());
  //timestamp = now();
  timestamp = Teensy3Clock.get();
  setTime(timestamp);

  char file_name_bin[30];
  char file_name_csv[30];

  sprintf(file_name_bin, "rawdata_%0d-%0d-%0d_%0d-%0d-%0d.BIN", year(), month(), day(), hour(), minute(), second());
  sprintf(file_name_csv, "metrics_%0d-%0d-%0d_%0d-%0d-%0d.csv", year(), month(), day(), hour(), minute(), second());

  if (!(SD.begin(SDCARD_CS_PIN))) {
    Serial.println("Unable to access the SD card");
  }else{
    if(config.write_raw_data){
      data_file = SD.open(file_name_bin, FILE_WRITE);
      data_file.write((byte*)&file_version, 2);
      data_file.write((byte*)&timestamp, 4);
      data_file.write((byte*)&send_num_fft_bins, 2);
      data_file.write((byte*)&iq_measurement, 1);
      data_file.write((byte*)&sample_rate, 2);
      data_file.flush();
    }
    if(write_csv_table){
      csv_file = SD.open(file_name_csv, FILE_WRITE);
      csv_file.println("timestamp, speed, speed_reverse, strength, strength_reverse, mean_amplitude, mean_amplitude_reverse, bins_with_signal, bins_with_signal_reverse, pedestrian_mean_amplitude");
      csv_file.flush();
    }
    write_sd = true;
  }

  // IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 |= IOMUXC_PAD_SPEED(0) | IOMUXC_PAD_DSE(1) | IOMUXC_PAD_SRE(0);
  // IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_01 |= IOMUXC_PAD_SPEED(0) | IOMUXC_PAD_DSE(1) | IOMUXC_PAD_SRE(0);
  // IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02 |= IOMUXC_PAD_SPEED(0) | IOMUXC_PAD_DSE(1) | IOMUXC_PAD_SRE(0);
  // IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 |= IOMUXC_PAD_SPEED(0) | IOMUXC_PAD_DSE(1) | IOMUXC_PAD_SRE(0);
  //IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 = 0b0000'0000'0000'0000'0001'0000'1000'1001;
}


void loop() {
  // TODO
  // in functions.h or somewhere else: void processInputs(Config& config);
  // here only remains: processInputs(config);

  // control input from serial
  if (Serial.available() > 0) {
    int input = Serial.read();

    if((input==0) & (mic_gain > 0.001)){ // - key
      mic_gain=mic_gain-0.01;
    }
    if((input==1) & (mic_gain<10)){ // + key
      mic_gain=mic_gain+0.01;
    }

    if(input==100){ // d for data
      send_output=true;
    }

    if(input==111){ //o
     alpha = alpha + 0.01;
    }
    if(input==108){ //l
      alpha = alpha - 0.01;
    }
    if(input==105){ //i
      psi=psi+0.01;
    }
    if(input==107){ //k
      psi=psi-0.01;
    }
    if(input==111 || input==108 || input==105 || input==107){
      SerialUSB1.print("alpha = ");
      SerialUSB1.println(alpha);
      SerialUSB1.print("psi = ");
      SerialUSB1.println(psi);

      //after https://www.faculty.ece.vt.edu/swe/argus/iqbal.pdf
      A = 1/alpha;
      C = -sin(psi)/(alpha*cos(psi));
      D = 1/cos(psi);
      I_gain.setGain(A);
      Q_mixer.gain(0, C);
      Q_mixer.gain(1, D);
    }

    if(input==84) { //T for time
      unsigned long pctime = Serial.parseInt();
      if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
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

  //generate output

  if(audio.hasData()){
    audioStuff.processData(audioResults);

    // save data on sd card
    if(write_sd){
      // elapsed time since start of sensor in milliseconds
      unsigned long time_millis = millis();
      if(write_raw_data){
        if(write_8bit){
          data_file.write((byte*)&time_millis, 4);
          for(i = send_min_fft_bin; i < send_max_fft_bin; i++){
            data_file.write((uint8_t)-spectrum[i]);
          }
        }else{
          data_file.write((byte*)&time_millis, 4);
          for(i = send_min_fft_bin; i < send_max_fft_bin; i++){
            data_file.write((byte*)&spectrum[i], 4);
          }
        }
        data_file.flush();
      }
      if(write_csv_table){
        csv_file.print(time_millis);
        csv_file.print(", ");
        csv_file.print(audioResults.detected_speed);
        csv_file.print(", ");
        csv_file.print(audioResults.detected_speed_reverse);
        csv_file.print(", ");
        csv_file.print(audioResults.max_amplitude);
        csv_file.print(", ");
        csv_file.print(audioResults.max_amplitude_reverse);
        csv_file.print(", ");
        csv_file.print(audioResults.mean_amplitude);
        csv_file.print(", ");
        csv_file.print(audioResults.mean_amplitude_reverse);
        csv_file.print(", ");
        csv_file.print(bins_with_signal);
        csv_file.print(", ");
        csv_file.print(bins_with_signal_reverse);
        csv_file.print(", ");
        csv_file.println(pedestrian_amplitude);
        csv_file.flush();
      }

      // SerialUSB1.print("csv sd write time: ");
      // SerialUSB1.println(millis()-time_millis);
    }

    // send data via Serial
    if(Serial && send_output){
      Serial.write((byte*)&mic_gain, 1);

      Serial.write((byte*)&max_freq_Index, 2);

      // highest peak-to-peak distance of the signal (if >= 1 clipping occurs)
      float peak = peak1.read();
      Serial.write((byte*)&peak, 4);

      Serial.write((byte*)&(send_num_fft_bins), 2);

      // send spectrum
      for(i = send_min_fft_bin; i < send_max_fft_bin; i++)
      {
        Serial.write((byte*)&audioResults.noise_floor_distance[i], 4);
      }

      send_output = false;
    }
  }
}


