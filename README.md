# SimpleBTVario
Simple Bluetooth Vario (Arduino + MS5611 barometric pressure sensor) with LK8000 bluetooth sentences

Forked from Vario by D.Felix (https://codebender.cc/sketch:34645#Vario.ino)

## Differences from original:
 + Use of MS5611 sensor instead of BMP085
 + Use of a button to set to sleep/awake the µP
 + Use of the native arduino tone functions (compatibility with ARM Cortex M0)

## Schematic / BOM
 + Arduino Pro Mini 3V
 + HC-06/05 (not needed if used a simple audio vario)
 + switch
 + piezo buzzer
 + Lipo Battery
 + Optional : USB LiPo battery charger
![Schema](SimpleBTVario_bb.png#3)

## :information_source: Compatibility with ARM Cortex M0 in progess
