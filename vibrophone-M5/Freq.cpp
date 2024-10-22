#include <M5StickC.h>
#include "AudioTools.h"

#include "Freq.h"
#include "config.h"

extern VolumeStream out;
extern AudioInfo info;

SineWaveGenerator<int16_t> sineWave(32000);
GeneratedSoundStream<int16_t> sound(sineWave);
StreamCopy copier(out, sound);
Task task("freq-copy", 10000, 1, 0);

void freq_enable()
{
  sineWave.begin(info, DEFAULT_FREQ);
  task.begin([]()
             { copier.copy(); });
}

void freq_disable()
{
  task.end();
  sineWave.end();
}

void freq_update(unsigned int freq)
{
  sineWave.begin(info, freq);
}

void freq_display()
{
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(2);

  M5.Lcd.println("Frequency");
  M5.Lcd.println("");

  M5.Lcd.setTextSize(3);

  M5.Lcd.print(freq);
  M5.Lcd.println("Hz");

  M5.Lcd.setTextSize(1);

  M5.Lcd.println("");
  M5.Lcd.println("");
  M5.Lcd.print("V");
  M5.Lcd.print(VERSION_NUMBER);
  M5.Lcd.print(" ");
  M5.Lcd.println("Press to switch mode");
}
