#include <M5StickC.h>
#include "AudioTools.h"

#include "Blue.h"
#include "Freq.h"
#include "config.h"


vibrophone_mode mode = DEFAULT_MODE;

AnalogAudioStream dac;
VolumeStream out(dac);

AudioInfo info;

void vibrophone_update_mode()
{
  M5.update();
  if (M5.BtnA.isPressed() && mode == vibrophone_mode::VIBRO_BLUETOOTH)
  {
    blue_disable();
    freq_enable();
    mode = vibrophone_mode::VIBRO_FREQUENCY;
  }
  else if (M5.BtnA.isPressed() && mode == vibrophone_mode::VIBRO_FREQUENCY)
  {
    freq_disable();
    blue_enable();
    mode = vibrophone_mode::VIBRO_BLUETOOTH;
  }
}

void vibrophone_update_output()
{
  unsigned int freq_control = map(analogRead(MOL2), 0, 4095, MIN_FREQ, MAX_FREQ);
  static unsigned int old_freq = 0;
  if (abs(static_cast<int>(freq_control - old_freq)) > 2) {
    if (mode == vibrophone_mode::VIBRO_BLUETOOTH)
    {
      blue_update(freq_control);
    }
    else if (mode == vibrophone_mode::VIBRO_FREQUENCY)
    {
      freq_update(freq_control);
    }
    old_freq = freq_control;
  }

  unsigned int vol_control = (analogRead(MOL1) * 100 / 4045);
  if (vol_control > 100)
    vol_control = 100;
  static unsigned int old_vol = 0;
  if (abs(static_cast<int>(vol_control - old_vol)) > 2) {
    out.setVolume((float)vol_control / (float)100);
    old_vol = vol_control;
    if (mode == vibrophone_mode::VIBRO_BLUETOOTH)
    {
      blue_display();
    }
    else if (mode == vibrophone_mode::VIBRO_FREQUENCY)
    {
      freq_display();
    }
  }
}

void setup() {
  Serial.begin(115200);
  AudioLogger::instance().begin(Serial, AudioLogger::Info);
  M5.begin();

  pinMode(MOL1, INPUT);
  pinMode(MOL2, INPUT);
  
  info = dac.audioInfo();
  dac.begin();
  out.begin(info);

  sleep(1); // Wait for lcd init

  if (mode == vibrophone_mode::VIBRO_FREQUENCY)
    blue_enable();
  else if (mode == vibrophone_mode::VIBRO_FREQUENCY)
    freq_enable();
}

void loop() {
  vibrophone_update_mode();
  vibrophone_update_output();
  delay(100);
}
