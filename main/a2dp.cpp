#define A2DP_LEGACY_I2S_SUPPORT false
#define A2DP_I2S_AUDIOTOOLS true

#include "AudioTools.h"
#include "BluetoothA2DPSink.h"

AnalogAudioStream dac;
VolumeStream out(dac);

AudioInfo info;

Equilizer3Bands eq(out);
ConfigEquilizer3Bands cfg_eq;
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
  static bool init = true;
  static String name;

  if (init)
  {
    a2dp_sink.set_stream_reader(a2dp_cb, false);
    a2dp_sink.set_avrc_connection_state_callback(blue_display);
    
    name = String(DEFAULT_BT_NAME) + String(" ") + String(VERSION_NUMBER);

    cfg_eq = eq.defaultConfig();
    cfg_eq.copyFrom(info);
    cfg_eq.gain_low = 1;
    cfg_eq.freq_low = DEFAULT_CROSSOVER;
    cfg_eq.gain_medium = 0;
    cfg_eq.gain_high = 0;

    init = false;
  }



  eq.begin(cfg_eq);

  a2dp_sink.start(name.c_str(), true);

  blue_display(false);
}

void blue_disable()
{
  a2dp_sink.end();
  eq.end();
}

void blue_update(unsigned int freq)
{
  cfg_eq.freq_low = freq;
}


extern "C" void app_main(void){
  info = dac.audioInfo();
  dac.begin();
  out.begin(info);
  blue_enable();

  vTaskDelete(NULL);
}

