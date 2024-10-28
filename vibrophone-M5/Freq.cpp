#include <M5StickC.h>
#include "AudioTools.h"

#include "Freq.h"
#include "config.h"

extern AnalogAudioStream dac;
extern VolumeStream out;
extern AudioInfo info;

SineWaveGenerator<int16_t> sineWave(16000);
GeneratedSoundStream<int16_t> sound(sineWave);
StreamCopy copier(dac, sound);
Task task("freq-copy", 10000, 1, 0);

unsigned int freq;

void freq_enable()
{
  freq = DEFAULT_FREQ;
  sineWave.begin(info, DEFAULT_FREQ);
  sound.begin(info);
  task.begin([]()
             { copier.copy(); });
  
  freq_display();
}

void freq_disable()
{
  task.end();
  sound.end();
  sineWave.end();
}

void freq_update(unsigned int f)
{
  sineWave.begin(info, f);
  freq = f;

  freq_display();
}

void freq_display()
{
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(1);

  auto vol = out.volume() * 100;
  for (int i = 0; i < vol / 10; i++)
  {
    M5.Lcd.print("-");
  }
  M5.Lcd.println("");

  M5.Lcd.setTextSize(2);

  M5.Lcd.println("Frequency");
  M5.Lcd.println("");

  M5.Lcd.setTextSize(3);

  M5.Lcd.print(freq);
  M5.Lcd.println("Hz");

  M5.Lcd.setTextSize(1);

  M5.Lcd.println("");
  M5.Lcd.print("V");
  M5.Lcd.print(VERSION_NUMBER);
  M5.Lcd.print(" ");
  M5.Lcd.println("Press to switch mode");
}
