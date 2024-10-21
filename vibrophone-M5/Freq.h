#pragma once

#include <M5StickC.h>
#include <DacESP32.h>

void freq_setup(unsigned int f);

void freq_enable();
void freq_disable();
void freq_update(unsigned int freq_control, unsigned int vol_control);
void freq_display();
