#include "Vibrophone.h"
#include "Freq.h"

DacESP32 dac1(GPIO_NUM_26);
unsigned int freq;
unsigned int vol;

void freq_setup(unsigned int f) {
  freq = f;
}

void freq_enable() {
  dac1.enable();
  dac1.outputCW(freq);
}

void freq_disable() {
  dac1.disable();
}

void freq_update(unsigned int freq_control, unsigned int vol_control) {
  freq = freq_control;
  static unsigned int old_freq = 0;
  if (freq != old_freq) {
    dac1.setCwFrequency(freq);
    old_freq = freq;
  }

  vol = vol_control;
  static unsigned int old_vol = 0;
  if (vol != old_vol) {
    dac_cw_scale_t scale = DAC_CW_SCALE_8;
    switch (vol / 25)
    {
      case 0:
        scale = DAC_CW_SCALE_1;
      break;
      case 1:
        scale = DAC_CW_SCALE_2;
      break;
      case 2:
        scale = DAC_CW_SCALE_4;
      break;
      case 3:
        scale = DAC_CW_SCALE_8;
      break;
    }

    dac1.setCwScale(scale);
    old_vol = vol;
  }
}

void freq_display() {
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
