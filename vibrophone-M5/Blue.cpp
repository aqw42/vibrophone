#include <Arduino.h>
#include <M5StickC.h>
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"

#include "Blue.h"
#include "config.h"

extern AnalogAudioStream dac;
extern VolumeStream out;
extern AudioInfo info;

Equilizer3Bands eq(out);
BluetoothA2DPSink a2dp_sink;

const int buffer_size = 80;

void a2dp_cb(const uint8_t *data, uint32_t length)
{
  int count = length / buffer_size + 1;
  for (int j = 0; j < count; j++)
  {
    const uint8_t *start = data + (j * buffer_size);
    const uint8_t *end = min(data + length, start + buffer_size);
    int len = end - start;
    if (len > 0)
    {
      eq.write(start, len);
    }
  }
}

void blue_enable()
{
  a2dp_sink.set_stream_reader(a2dp_cb, false);

  eq.setOutput(dac);
  auto &cfg_eq = eq.defaultConfig();
  cfg_eq.copyFrom(info);

  cfg_eq.gain_low = 1;
  cfg_eq.freq_low = DEFAULT_CROSSOVER;
  cfg_eq.gain_medium = 0;
  cfg_eq.gain_high = 0;

  eq.begin(cfg_eq);

  String name = String(DEFAULT_BT_NAME) + String(" ") + String(VERSION_NUMBER);
  a2dp_sink.start(name.c_str(), true);
}

void blue_disable()
{
  a2dp_sink.end();
  eq.end();
}

void blue_update(unsigned int freq)
{
  auto &cfg = eq.config();
  cfg.freq_low = freq;
  eq.begin(cfg);
}

void blue_display()
{
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