#include "Blue.h"
#include "Vibrophone.h"

AnalogAudioStream out;
String name;
BluetoothA2DPSink a2dp_sink(out);
unsigned int blue_freq;
unsigned int blue_vol;


void blue_setup(String n) {
  name = n;
}

void blue_enable() {
  a2dp_sink.start(name.c_str(), true);
}

void blue_disable() {
  a2dp_sink.end();
}

void blue_update(unsigned int freq_control, unsigned int vol_control) {
  blue_freq = freq_control;
  static unsigned int old_freq = 0;
  if (blue_freq != old_freq) {
    //dac1.setCwFrequency(freq);
    old_freq = blue_freq;
  }

  blue_vol = vol_control;
  static unsigned int old_vol = 0;
  if (blue_vol != old_vol) {
    a2dp_sink.set_volume(blue_vol);
    old_vol = blue_vol;
  }
}

void blue_display() {
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
  M5.Lcd.print("V");
  M5.Lcd.print(VERSION_NUMBER);
  M5.Lcd.print(" ");
  M5.Lcd.println("Press to switch mode");
}