#include "AudioSystem.h"
#include "Config.h"
#include "functions.h"

#include <SD.h>
#include <SPI.h>
#include <SerialFlash.h>
#include <TimeLib.h>
#include <Wire.h>
#include <utility/imxrt_hw.h>

// Audio shield SD card:
#define SDCARD_CS_PIN 10
#define SDCARD_MOSI_PIN 11 // Teensy 4 ignores this, uses pin 11
#define SDCARD_SCK_PIN 13  // Teensy 4 ignores this, uses pin 13

// builtin SD card:
// #define SDCARD_CS_PIN    BUILTIN_SDCARD
// #define SDCARD_MOSI_PIN  11  // not actually used
// #define SDCARD_SCK_PIN   13  // not actually used

// variables for file stuff
File data_file;              // file for raw data
File csv_file;               // file for metrics as csv
time_t timestamp;            // time stamp for saved data
uint16_t file_version = 1.1; // file version of the output

AudioSystem audio;
AudioSystem::Results audioResults; // global to optimize for speed
Config config;

void setup()
{
    setSyncProvider(getTeensy3Time);
    setI2SFreq(config.audio.sample_rate);

    // TODO maybe move to audio
    pinMode(PIN_A3, OUTPUT); // A3=17, A8=22, A4=18
    digitalWrite(PIN_A4, LOW);

    audio.setup(config.audio, config.max_pedestrian_speed, config.send_max_speed);

    // TODO
    // sdCard.setup();

    //============== SD card =============

    // Configure SPI
    SPI.setMOSI(SDCARD_MOSI_PIN);
    SPI.setSCK(SDCARD_SCK_PIN);

    // setTime(Teensy3Clock.get());
    // timestamp = now();
    timestamp = Teensy3Clock.get();
    setTime(timestamp);

    char file_name_bin[30];
    char file_name_csv[30];

    sprintf(file_name_bin, "rawdata_%0d-%0d-%0d_%0d-%0d-%0d.BIN", year(), month(), day(), hour(), minute(), second());
    sprintf(file_name_csv, "metrics_%0d-%0d-%0d_%0d-%0d-%0d.csv", year(), month(), day(), hour(), minute(), second());

    if(!(SD.begin(SDCARD_CS_PIN)))
    {
        Serial.println("Unable to access the SD card");
    }
    else
    {
        if(config.write_raw_data)
        {
            auto const binCount = audio.getNumberOfFftBins();

            data_file = SD.open(file_name_bin, FILE_WRITE);
            data_file.write((byte*)&file_version, 2);
            data_file.write((byte*)&timestamp, 4);
            data_file.write((byte*)&binCount, 2);
            data_file.write((byte*)&config.audio.iq_measurement, 1);
            data_file.write((byte*)&config.audio.sample_rate, 2);
            data_file.flush();
        }
        if(config.write_csv_table)
        {
            csv_file = SD.open(file_name_csv, FILE_WRITE);
            csv_file.println("timestamp, speed, speed_reverse, strength, strength_reverse, "
                             "mean_amplitude, mean_amplitude_reverse, bins_with_signal, "
                             "bins_with_signal_reverse, pedestrian_mean_amplitude");
            csv_file.flush();
        }
    }

    // IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 |= IOMUXC_PAD_SPEED(0) | IOMUXC_PAD_DSE(1)
    // | IOMUXC_PAD_SRE(0); IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_01 |=
    // IOMUXC_PAD_SPEED(0) | IOMUXC_PAD_DSE(1) | IOMUXC_PAD_SRE(0);
    // IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02 |= IOMUXC_PAD_SPEED(0) | IOMUXC_PAD_DSE(1)
    // | IOMUXC_PAD_SRE(0); IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 |=
    // IOMUXC_PAD_SPEED(0) | IOMUXC_PAD_DSE(1) | IOMUXC_PAD_SRE(0);
    // IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 =
    // 0b0000'0000'0000'0000'0001'0000'1000'1001;
}

void loop()
{
    bool sendOutput = false; // send output over serial? (ONCE)

    processInputs(config.audio, sendOutput);
    if(config.audio.hasChanges)
    {
        audio.updateIQ(config.audio);
        config.audio.hasChanges = false;
    }

    // generate output
    if(audio.hasData())
    {
        audio.processData(audioResults);

        // save data on sd card
        if(config.write_sd)
        {
            // elapsed time since start of sensor in milliseconds
            unsigned long time_millis = millis();
            if(config.write_raw_data)
            {
                data_file.write((byte*)&time_millis, 4);

                for(int i = audioResults.minBinIndex; i < audioResults.maxBinIndex; i++)
                    if(config.write_8bit)
                        data_file.write((uint8_t)-audioResults.spectrum[i]);
                    else
                        data_file.write((byte*)&audioResults.spectrum[i], 4);

                data_file.flush();
            }
            if(config.write_csv_table)
            {
                csv_file.print(time_millis);
                csv_file.print(", ");
                csv_file.print(audioResults.detected_speed);
                csv_file.print(", ");
                csv_file.print(audioResults.detected_speed_reverse);
                csv_file.print(", ");
                csv_file.print(audioResults.amplitudeMax);
                csv_file.print(", ");
                csv_file.print(audioResults.amplitudeMaxReverse);
                csv_file.print(", ");
                csv_file.print(audioResults.mean_amplitude);
                csv_file.print(", ");
                csv_file.print(audioResults.mean_amplitude_reverse);
                csv_file.print(", ");
                csv_file.print(audioResults.bins_with_signal);
                csv_file.print(", ");
                csv_file.print(audioResults.bins_with_signal_reverse);
                csv_file.print(", ");
                csv_file.println(audioResults.pedestrian_amplitude);
                csv_file.flush();
            }

            // SerialUSB1.print("csv sd write time: ");
            // SerialUSB1.println(millis()-time_millis);
        }

        // send data via Serial
        if(Serial && sendOutput)
        {
            Serial.write((byte*)&config.audio.mic_gain, 1);
            Serial.write((byte*)&audioResults.max_freq_Index, 2);

            // highest peak-to-peak distance of the signal (if >= 1 clipping occurs)
            float peak = audio.getPeak();
            Serial.write((byte*)&peak, 4);

            auto const binCount = audio.getNumberOfFftBins();
            Serial.write((byte*)&binCount, 2);

            for(size_t i = audioResults.minBinIndex; i < audioResults.maxBinIndex; i++)
                Serial.write((byte*)&audioResults.noise_floor_distance[i], 4);
        }
    }
}
