# SimpleBTVario
Simple Bluetooth Vario (Arduino + MS5611 barometric pressure sensor) with LK8000/LXNAV bluetooth sentences

Forked from Vario by D.Felix (https://codebender.cc/sketch:34645#Vario.ino)

<a href="https://drive.google.com/uc?export=view&id=1Vi6WSBSU8t0mJlr0S6Y0-SK6L4gqkTsE"><img src="https://drive.google.com/uc?export=view&id=1Vi6WSBSU8t0mJlr0S6Y0-SK6L4gqkTsE" width="200" /></a>

## Differences from original:
 + Use of MS5611 sensor instead of BMP085
 + Use of a button to set to sleep/awake the µP
 + Use of the native arduino tone functions (compatibility with ARM Cortex M0)
 + Support LXNAV sentences (now work for XC Soar along with LK8000 for XC Track)

## Schematic / BOM
 + Arduino Pro Mini 3V
 + MS5611
 + HC-06/05 (not needed if used as a simple audio vario)
 + switch
 + piezo buzzer
 + Lipo Battery
 + Optional : USB LiPo battery charger
![Schema](SimpleBTVario_bb.png#3)

## :information_source: Compatibility with ARM Cortex M0 in progess
