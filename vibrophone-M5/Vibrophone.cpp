#include "Vibrophone.h"

Vibrophone::Vibrophone(vibrophone_mode functionning_mode)
 : mode(functionning_mode) {
  pinMode(MOL1, INPUT);
  pinMode(MOL2, INPUT);
}

void Vibrophone::start(unsigned int default_frequency, String bluetooth_name) {
  M5.begin();
  
  blue_setup("Vibro Machin");
  freq_setup(default_frequency);
  
  if (mode == vibrophone_mode::VIBRO_FREQUENCY)
    blue_enable();
  else if (mode == vibrophone_mode::VIBRO_FREQUENCY)
    freq_enable();
}

void Vibrophone::update_mode() {
  M5.update();
  if (M5.BtnA.isPressed() && mode == vibrophone_mode::VIBRO_BLUETOOTH) {
    blue_disable();
    freq_enable();
    mode = vibrophone_mode::VIBRO_FREQUENCY;
  } else if (M5.BtnA.isPressed() && mode == vibrophone_mode::VIBRO_FREQUENCY) {
    freq_disable();
    blue_enable();
    mode = vibrophone_mode::VIBRO_BLUETOOTH;
  }
}

void Vibrophone::update_output() {
  unsigned int freq_control = (analogRead(MOL2) * 90 / 4045) + 20;
  unsigned int vol_control = analogRead(MOL1) * 100 / 4045;
  if (mode == vibrophone_mode::VIBRO_BLUETOOTH) {
    blue_update(freq_control, vol_control);
    blue_display();
  } else if (mode == vibrophone_mode::VIBRO_FREQUENCY) {
    freq_update(freq_control, vol_control);
    freq_display();
  }

}