"C:\Users\coren\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\4.6/esptool.exe" --chip esp32 --port "COM3" --baud 1500000  --before default_reset --after hard_reset write_flash -e -z --flash_mode keep --flash_freq keep --flash_size keep 0x1000 "c:\Users\coren\Documents\Arduino\vibrophone\vibrophone-M5\build/vibrophone-M5.ino.bootloader.bin" 0x8000 "c:\Users\coren\Documents\Arduino\vibrophone\vibrophone-M5\build/vibrophone-M5.ino.partitions.bin" 0xe000 "C:\Users\coren\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.0.5/tools/partitions/boot_app0.bin" 0x10000 "c:\Users\coren\Documents\Arduino\vibrophone\vibrophone-M5\build/vibrophone-M5.ino.bin" 