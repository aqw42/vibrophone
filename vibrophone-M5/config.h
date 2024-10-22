#pragma once

#define MOL1 0
#define MOL2 36

/* CHANGELOG
0.1 : First version, freq+bt, no controls, no crossover
0.2 : Rework, basic functionning
0.3 : Volume and freq Controls for generator and a2dp, no crossver 
0.4 : Crossover implemented
0.5 : Rework / polishing, make the frequency generator use audiotools library
0.6 : More polishing, adding global volume control
*/
#define VERSION_NUMBER 0.6

enum class vibrophone_mode : bool
{
  VIBRO_FREQUENCY,
  VIBRO_BLUETOOTH
};

#define DEFAULT_MODE vibrophone_mode::VIBRO_BLUETOOTH

#define MIN_FREQ 1
#define MAX_FREQ 100

#define DEFAULT_CROSSOVER 100
#define DEFAULT_FREQ 100

#define DEFAULT_BT_NAME "Vibro"