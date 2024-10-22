#pragma once

#include <M5StickC.h>

#include "Freq.h"
#include "Blue.h"

#define MOL1 0
#define MOL2 36

/* CHANGELOG
0.1 : First version, freq+bt, no controls, no crossover
0.2 : Rework, basic functionning
0.3 : Volume and freq Controls for generator and a2dp, no crossver 
0.4 : Crossover implemented
0.5 : Rework / polishing, make the frequency generator use audiotools library
*/
#define VERSION_NUMBER 0.4

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
