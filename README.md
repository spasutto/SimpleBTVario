# SimpleBTVario
Simple Bluetooth Vario (Arduino + MS5611 barometric pressure sensor) with LK8000/LXNAV bluetooth sentences

Forked from Vario by D.Felix (https://codebender.cc/sketch:34645#Vario.ino)

<a href="https://drive.google.com/uc?export=view&id=1Vi6WSBSU8t0mJlr0S6Y0-SK6L4gqkTsE"><img src="https://drive.google.com/uc?export=view&id=1Vi6WSBSU8t0mJlr0S6Y0-SK6L4gqkTsE" width="200" /></a>

## Differences from original:
 + Use of MS5611 sensor instead of BMP085
 + Use of a button to set to sleep/awake the µP
 + Use of the native arduino tone functions (compatibility with ARM Cortex M0)
 + Support LXNAV sentences (now work for XC Soar along with LK8000 for XC Track)
 + Support simple NMEA-like sentences from user (useable with any Android bluetooth terminal app) :
   + "$ALTI=400" : set current altitude to 400m (pressure compensation)
   + "$BTMODE=LXNAV" : set bluetooth sentences to LXNAV or LK8000
   + "$RESET" : reset the conf to default values

## Schematic / BOM
 + Arduino Pro Mini **3.3V**
 + MS5611
 + HC-06/05 (not needed if used as a simple audio vario)
 + switch, either ;
   + spdt if connected between battery charger and arduino
   + push button if connected to D2 (set µP in sleep mode)
 + piezo buzzer
 + Lipo Battery
 + Optional : USB LiPo battery charger
![Schema](SimpleBTVario_bb.png#4)

## Programming/assembly instructions ##

First, use a 3.3V Arduino Pro Mini, because the 3.7V battery is connected directly to the VCC and the atmega328p is not designed to run @16MHz with Vcc around 3.3V. The battery (or battery charger output) must be connected to VCC and not RAW or the battery level will be wrong.

To measure properly the battery voltage, you must determine the real voltage of the 1.1V internal reference voltage (ref [Nick Gammon site](https://www.gammon.com.au/adc)). So first upload this simple code to the arduino :

    void setup ()
    {
      ADMUX = bit (REFS0) | bit (REFS1);  // Internal 1.1V reference
    }
    void loop () { }

And then, with the help of a multimeter, measure the voltage at Aref pin (18 on pro mini), it should be ~1.1V. Finally, update the value of the constant variable named "InternalReferenceVoltage" in the sketch SimpleBTVario.ino and upload it, you're done with the programming part!!!

### Low power enhancement ###
You can remove the regulator and the led because they use power, and also the built-in led. Similarly you can remove the leds on the HC-05/6 and MS5611.
