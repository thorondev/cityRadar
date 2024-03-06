#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "AudioSystem.h"

#include <TimeLib.h>

void setI2SFreq(int freq);
void printDigits(int digits);
time_t getTeensy3Time();

void processInputs(AudioSystem::Config& config);

#endif
