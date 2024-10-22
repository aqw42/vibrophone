#include <Arduino.h>
#include "Blue.h"
#include "Vibrophone.h"

String name;

AnalogAudioStream out;
Equilizer3Bands eq(out);
BluetoothA2DPSink a2dp_sink;

unsigned int blue_freq;
unsigned int blue_vol;

void a2dp_cb(const uint8_t *data, uint32_t len)
{
  eq.write(data, len);
}

void blue_setup(String n) {
  name = n;
  a2dp_sink.set_stream_reader(a2dp_cb, false);
}

void blue_enable() {
  auto cfg_out = out.defaultConfig();
  cfg_out.channels = 2;
  cfg_out.bits_per_sample = 16;
  cfg_out.sample_rate = 44100;
  out.begin(cfg_out);

  auto &cfg_eq = eq.defaultConfig();
  cfg_eq.gain_low = 1;
  cfg_eq.freq_low = 100;
  cfg_eq.gain_medium = 0;
  cfg_eq.gain_high = 0;
  cfg_eq.channels = 2;
  cfg_eq.bits_per_sample = 16;
  cfg_eq.sample_rate = 44100;
  eq.begin(cfg_eq);

  a2dp_sink.start(name.c_str(), true);
}

void blue_disable() {
  a2dp_sink.end();
  eq.end();
  out.end();
}

void blue_update(unsigned int freq_control, unsigned int vol_control) {
  blue_freq = freq_control;
  static unsigned int old_freq = 0;
  if (abs(static_cast<int>(blue_freq - old_freq)) > 2) {
    auto &cfg = eq.config();
    cfg.freq_low = blue_freq;
    eq.begin(cfg);
    old_freq = blue_freq;
  }

  blue_vol = vol_control;
  static unsigned int old_vol = 0;
  if (abs(static_cast<int>(blue_vol - old_vol)) > 2) {
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