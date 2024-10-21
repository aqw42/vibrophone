#pragma once

#include <M5StickC.h>

#include "Freq.h"
#include "Blue.h"

#define MOL1 0
#define MOL2 36

#define VERSION_NUMBER 0.3

enum class vibrophone_mode : bool
{
  VIBRO_FREQUENCY,
  VIBRO_BLUETOOTH
};

class Vibrophone {
private:
  vibrophone_mode mode;

public:
  Vibrophone(vibrophone_mode functionning_mode);

  void start(unsigned int default_frequency, String bluetooth_name);
  void update_mode();
  void update_output();
};
