#include "Vibrophone.h"

Vibrophone *vibro;

void setup() {
  vibro = new Vibrophone(vibrophone_mode::VIBRO_FREQUENCY, 60, "VIBRO");
}

void loop() {
  vibro->update_mode();
  vibro->update_output();
  delay(100);
}
