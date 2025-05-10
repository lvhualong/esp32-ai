#!/bin/sh

# 烧录固件到ESP32开发板，注意固件版本 和 串口号
esptool.py -p /dev/ttyACM0 -b 2000000 write_flash 0 ../releases/v0.9.9_bread-compact-wifi/merged-binary.bin
