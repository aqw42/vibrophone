#include "Vibrophone.h"

Vibrophone::Vibrophone(vibrophone_mode functionning_mode, unsigned int default_frequency, String bluetooth_name)
  : mode(functionning_mode),
    freq(Freq{ default_frequency, GPIO_NUM_25 }),
    blue(Blue{ bluetooth_name }) {
  pinMode(MOL1, INPUT);
  pinMode(MOL2, INPUT);
  pinMode(MODE_BUT, INPUT);
}

void Vibrophone::update_mode() {
  if (digitalRead(MODE_BUT) && mode == vibrophone_mode::VIBRO_BLUETOOTH) {
    blue.disable();
    freq.enable();
    mode = vibrophone_mode::VIBRO_FREQUENCY;
  } else if (digitalRead(MODE_BUT) && mode == vibrophone_mode::VIBRO_FREQUENCY) {
    freq.disable();
    blue.enable();
    mode = vibrophone_mode::VIBRO_BLUETOOTH;
  }
}

void Vibrophone::update_output() {
  if (mode == vibrophone_mode::VIBRO_BLUETOOTH) {
    blue.update();
    blue.display();
  } else if (mode == vibrophone_mode::VIBRO_FREQUENCY) {
    freq.update((analogRead(MOL1) * 50 / 2045) + 30);
    freq.display();
  }
}