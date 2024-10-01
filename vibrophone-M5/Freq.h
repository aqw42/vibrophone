#pragma once

#include <M5StickC.h>
#include <DacESP32.h>


class Freq {
private:
  unsigned int freq;
  DacESP32 dac1;

public:
  Freq(unsigned int f, gpio_num_t pin);
  Freq(const Freq &) = delete;
  Freq &operator=(const Freq &) = delete;
  Freq(Freq &&);
  Freq &operator=(Freq &&) = delete;

  void enable();
  void disable();
  void update(unsigned int freq);
  void display();
};
