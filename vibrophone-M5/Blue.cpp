#include "Blue.h"

Blue::Blue(String n)
  : name(n), a2dp_sink(BluetoothA2DPSink(out)) {}

Blue::Blue(Blue &&other)
  : name(other.name), a2dp_sink(other.a2dp_sink) {}

void Blue::enable() {
  a2dp_sink.start(name.c_str(), true);
}

void Blue::disable() {
  a2dp_sink.end();
}

void Blue::update() {
}

void Blue::display() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(2);

  M5.Lcd.println("Bluetooth");
  M5.Lcd.println("");


  M5.Lcd.setTextSize(2);

  if (a2dp_sink.is_connected())
    M5.Lcd.print("Connected");
  else
    M5.Lcd.print("Not connected");

  M5.Lcd.setTextSize(1);

  M5.Lcd.println("");
  M5.Lcd.println("");
  M5.Lcd.println("");
  M5.Lcd.println("Press to switch mode");
}