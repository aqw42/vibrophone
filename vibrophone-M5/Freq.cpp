#include "Freq.h"

Freq::Freq(unsigned int f, gpio_num_t pin)
  : freq(f), dac1(DacESP32(pin)) {}

Freq::Freq(Freq &&other) 
  : freq(other.freq), dac1(other.dac1) {}

void Freq::enable() {
  dac1.enable();
  dac1.outputCW(freq);
}

void Freq::disable() {
  dac1.disable();
}

void Freq::update(unsigned int freq_control) {
  freq = freq_control;
  static unsigned int old_freq = 0;
  if (freq != old_freq)
  {
    dac1.outputCW(freq);
    old_freq = freq;
  }
}

void Freq::display() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(2);

  M5.Lcd.println("Frequency");
  M5.Lcd.println("");


  M5.Lcd.setTextSize(3);

  M5.Lcd.print(freq);
  M5.Lcd.println("Hz");

  M5.Lcd.setTextSize(1);

  M5.Lcd.println("");
  M5.Lcd.println("");
  M5.Lcd.println("Press to switch mode");
}
