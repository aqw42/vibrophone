#pragma once

#include <M5StickC.h>
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"

void blue_setup(String n);
void blue_enable();
void blue_disable();
void blue_update(unsigned int freq_control, unsigned int vol_control);
void blue_display();
