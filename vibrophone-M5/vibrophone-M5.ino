#include "Vibrophone.h"

Vibrophone vibro(vibrophone_mode::VIBRO_BLUETOOTH);

void setup() {
  String name = String("Vibrophone") + String(" ") + WiFi.macAddress();
  vibro.start(60, name);
}

void loop() {
  vibro.update_mode();
  vibro.update_output();
  delay(100);
}
