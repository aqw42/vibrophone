#pragma once

#include <M5StickC.h>
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"

class Blue {
private:
  AnalogAudioStream out;
  String name;
  BluetoothA2DPSink a2dp_sink;

public:
  Blue(String n);
  Blue(const Blue &)            = delete;
  Blue &operator=(const Blue &) = delete;
  Blue(Blue &&);
  Blue &operator=(Blue &&)      = delete;

  void enable();
  void disable();
  void update();
  void display();
};