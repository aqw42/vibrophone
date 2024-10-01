#pragma once

#include <M5StickC.h>

#include "Freq.h"
#include "Blue.h"

#define MOL1 0
#define MOL2 36

enum class vibrophone_mode : bool
{
  VIBRO_FREQUENCY,
  VIBRO_BLUETOOTH
};

class Vibrophone {
private:
  vibrophone_mode mode;
  Freq freq;
  Blue blue;

public:
  Vibrophone(vibrophone_mode functionning_mode, unsigned int default_frequency, String bluetooth_name);

  void update_mode();
  void update_output();
};
