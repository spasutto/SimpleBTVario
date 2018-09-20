/*
 * 
SimpleBTVario by Sylvain Pasutto 2018 distributed under GNU General Public License

Forked from :

Arduino Vario by dfelix 2013

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


LK8000 integration based on Arduino Vario by Jaros, 2012
Arduino board creates NMEA like protocol with variometer output and beping sound.
LK8000 EXTERNAL INSTRUMENT SERIES 1 - NMEA SENTENCE: LK8EX1
VERSION A, 110217

$LK8EX1,pressure,altitude,vario,temperature,battery,*checksum

Field 0, raw pressure in hPascal:hPA*100 (example for 1013.25 becomes 101325)
no padding (987.25 becomes 98725, NOT 098725)
If no pressure available, send 999999 (6 times 9)
If pressure is available, field 1 altitude will be ignored
Field 1, altitude in meters, relative to QNH 1013.25
If raw pressure is available, this value will be IGNORED (you can set it to 99999
but not really needed)!(if you want to use this value, set raw pressure to 999999)
This value is relative to sea level (QNE). We are assuming that currently at 0m
altitude pressure is standard 1013.25.If you cannot send raw altitude, then send
what you have but then you must NOT adjust it from Basic Setting in LK.
Altitude can be negative. If altitude not available, and Pressure not available, set Altitude
to 99999. LK will say "Baro altitude available" if one of fields 0 and 1 is available.
Field 2, vario in cm/s
If vario not available, send 9999. Value can also be negative.
Field 3, temperature in C , can be also negative.If not available, send 99
Field 4, battery voltage or charge percentage.Cannot be negative.If not available, send 999.
Voltage is sent as float value like: 0.1 1.4 2.3 11.2. To send percentage, add 1000.
Example 0% = 1000. 14% = 1014 . Do not send float values for percentages.
Percentage should be 0 to 100, with no decimals, added by 1000!

Credits:
(1) http://code.google.com/p/bmp085driver/                             //bmp085 library
(2) http://mbed.org/users/tkreyche/notebook/bmp085-pressure-sensor/    //more about bmp085 and average filter
(3) http://code.google.com/p/rogue-code/                               //helpfull tone library to make nice beeping without using delay
(4) http://www.daqq.eu/index.php?show=prj_sanity_nullifier             //how to make loud piezo speaker
(5) http://lk8000.it                                                   //everything because of that
(6) http://taturno.com/2011/10/30/variometro-la-rivincita/             //huge thanks for Vario algorithm
(7) http://code.google.com/p/tinkerit/wiki/SecretVoltmeter             //how to measure battery level using AVR ucontroller
*/

#include <EEPROM.h>
#include <Wire.h>                      //i2c library
//#include "BMP085.h"                    //bmp085 library, download from url link (1)
#include <MS5611.h>
#if defined(ARDUINO_SAMD_ZERO)
#include <avr/dtostrf.h>                //we need that to use dtostrf() and convert float to string
#else
#include <stdlib.h>                     //we need that to use dtostrf() and convert float to string
#include <avr/sleep.h>
#endif

#define xstr(a) str(a)
#define str(a) #a

/////////////////////////////////////////
///////////////////////////////////////// variables that You can test and try
short speaker_pin = 8;                //arduino speaker output
short button_pin = 2;                //power off button
float vario_climb_rate_start = 0.4;    //minimum climb beeping value(ex. start climbing beeping at 0.4m/s)
float vario_sink_rate_start = -1.1;    //maximum sink beeping value (ex. start sink beep at -1.1m/s)
#define VARIO_VERSION 1.0
#define SAMPLES_ARR 6                  //define moving average filter array size (2->30), more means vario is less sensitive and slower
#define UART_SPEED 9600                //define serial transmision speed (9600,19200, etc...)
//#define BLINK_LED                      //if we blink led at battery reading
#define PERIOD_NMEA  333               //period for sending LK8000 sentences
#define PERIOD_BAT  1000               //period for checking battery level
#define VBATPIN A7                     //M0 VBat reading pin
/////////////////////////////////////////
/////////////////////////////////////////
//BMP085   bmp085 = BMP085();            //set up bmp085 sensor
MS5611 ms5611;
double     Temperature = 0.0f;
long     Pressure = 101325;
float    Altitude;
unsigned long     average_pressure;
int      Battery_Vcc = 0;             //variable to hold the value of Vcc from battery
const float P0 = 101325;              //Pressure at sea level (Pa)
const float InternalReferenceVoltage = 1.103; // as measured on my 3.3V arduino
//unsigned long get_time1 = millis();
unsigned long get_time2 = millis() + PERIOD_BAT;
unsigned long get_time3 = millis() + PERIOD_NMEA;
#ifdef BLINK_LED
boolean  light_on = false;
#endif
boolean  thermalling = false;
int      my_temperature = 1;
char     altitude_arr[6];            //wee need this array to translate float to string
char     vario_arr[5];               //wee need this array to translate float to string
int      samples = 40;
int      maxsamples = 50;
float    alt[51];
float    tim[51];
float    beep;
float    Beep_period;
float tempo;
float vario;
float N1;
float N2;
float N3;
float D1;
float D2;
static unsigned long k[SAMPLES_ARR];
static unsigned long battper[SAMPLES_ARR];
char serrecv, serbuff[32], *serptr = (serbuff + sizeof(serbuff));
int buttonState = LOW, lastButtonState = LOW;         // variable for reading the pushbutton status
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

enum TYPE_BT_SENTENCES
{
  LK8000,
  LXNAV
};
typedef struct
{
  char signature[11] = "BTVARIO" xstr(VARIO_VERSION);
  float p0 = 101325;              //Pressure at sea level (Pa)
  TYPE_BT_SENTENCES type_sentences = TYPE_BT_SENTENCES::LK8000;
  
} SETTINGS;
SETTINGS settings;

bool debounce(int pin, int *state, int *laststate, byte aimstate = HIGH);

template <typename T> unsigned long Averaging_Filter(unsigned long* values, T input) // moving average filter function
{
  unsigned long sum = 0;
  for (int i = 0; i < SAMPLES_ARR; i++)
  {
    values[i] = values[i + 1];
  }
  values[SAMPLES_ARR - 1] = input;
  for (int i = 0; i < SAMPLES_ARR; i++)
  {
    sum += values[i];
  }
  return ( sum / SAMPLES_ARR ) ;
}

void play_melody(bool intro = true)                 //play welcome beep after turning on arduino vario
{
  if (intro)
  {
    for (int aa = 300; aa <= 1500; aa = aa + 100)
    {
      tone(speaker_pin, aa, 20);            // play beep on pin 8 (note,duration)
      delay(10);
    }
  }
  for (int aa = 1500; aa >= 100; aa = aa - 100)
  {
    tone(speaker_pin, aa, 20);            // play beep on pin 8 (note,duration)
    delay(10);
  }
}

float readVcc()                         // function to read battery value
{
#if defined(ARDUINO_SAMD_ZERO)
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3f;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to V
  //Serial.print("VBat: " ); Serial.print(measuredvbat); Serial.println(" V");
#else
  ADCSRA =  bit (ADEN);   // turn ADC on
  ADCSRA |= bit (ADPS0) |  bit (ADPS1) | bit (ADPS2);  // Prescaler of 128
  ADMUX = bit (REFS0) | bit (MUX3) | bit (MUX2) | bit (MUX1);
  
  delay (10);  // let it stabilize
  
  bitSet (ADCSRA, ADSC);  // start a conversion  
  while (bit_is_set(ADCSRA, ADSC));
  
  return InternalReferenceVoltage / float (ADC + 0.5f) * 1024.0f; 
#endif
}

#if !defined(ARDUINO_SAMD_ZERO)
bool awakening = true;
void goToSleep()         // sleep µP if pin 2 turn from LOW to HIGH
{
  byte adcsra, mcucr1, mcucr2;
  play_melody(false);
  delay(250);
  makePinsInput();
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  EIMSK = _BV(INT0);             //enable INT0
  adcsra = ADCSRA;               //save the ADC Control and Status Register A
  ADCSRA = 0;                    //disable ADC
  cli();
  mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);  //turn off the brown-out detector
  mcucr2 = mcucr1 & ~_BV(BODSE);
  MCUCR = mcucr1;                //timed sequence
  MCUCR = mcucr2;                //BODS stays active for 3 cycles, sleep instruction must be executed while it's active
  sei();                         //ensure interrupts enabled so we can wake up again
  sleep_cpu();                   //go to sleep
  sleep_disable();               //wake up here
  ADCSRA = adcsra;               //restore ADCSRA
  makePinsInput();
  play_melody(true);
}

ISR(INT0_vect)         // wake up interrupt
{
  EIMSK = 0;                     //disable interrupts (only need one to wake up)
  awakening = true;
}

void makePinsInput(void)     //make all pins input pins with pullup resistors to minimize power consumption
{
  for (byte i=0; i<20; i++) {
    pinMode(i, INPUT_PULLUP);  
  }
//#ifdef BLINK_LED   // if the led isn't removed from the pcb, it is lit but dimmed by the internal pull up resistor
  pinMode(LED_BUILTIN, OUTPUT);          //except the LED pin
  digitalWrite(LED_BUILTIN, LOW);
//#endif
}
#endif // !defined(ARDUINO_SAMD_ZERO)

double getPressure(double altitude, double sealevelpressure)       // calculate pressure from given altitude and pressure at sea level
{
  return sealevelpressure * pow(1-altitude/44330.0f, 1.0f/0.1902949f);
}

void ParseCommand()        // parse simple NMEA-like commands
{
  serrecv = Serial.read();
  // reinit serbuff/serptr
  if (serptr >= (serbuff + sizeof(serbuff)))
  {
    serptr = &serbuff[sizeof(serbuff)];
    do
      *(--serptr) = 0;
    while (serptr > serbuff);
    if (serrecv == 0x0D || serrecv == 0x0A)
      return;
  }
  switch (serrecv)
  {
    case 0x0D:
    case 0x0A:
    if (strncmp(serbuff, "$HELP", 5) == 0)
    {
      Serial.println(F("\"$RESET\" : reset all settings"));
      Serial.println(F("\"$ALTI=450\" : set the current altitude to 450m"));
      Serial.println(F("\"$BTMODE=LXNAV\" : set the bluetooth sentences to LXNAV mode"));
      Serial.println(F("                           Supported values :"));
      Serial.println(F("                             - LXNAV"));
      Serial.println(F("                             - LK8000"));
    }
    else if (strncmp(serbuff, "$RESET", 6) == 0)
    {
      settings.type_sentences = TYPE_BT_SENTENCES::LK8000;
      settings.p0 = P0;
      saveConf();
    }
    else if (strncmp(serbuff, "$ALTI=", 6) == 0)
    {
      settings.p0 = ms5611.getSeaLevel(ms5611.readPressure(true), (float)atoi(serbuff + 6));
      saveConf();
    }
    else if (strncmp(serbuff, "$BTMODE=", 8) == 0)
    {
      if (strncmp(serbuff + 8, "LK8000", 6) == 0)
      {
        settings.type_sentences = TYPE_BT_SENTENCES::LK8000;
        saveConf();
      }
      else if (strncmp(serbuff + 8, "LXNAV", 5) == 0)
      {
        settings.type_sentences = TYPE_BT_SENTENCES::LXNAV;
        saveConf();
      }
    }
    serptr = (serbuff + sizeof(serbuff)); // reinit à la prochain iteration
    break;
    case '$':
    serptr = serbuff;
    default:
    *(serptr++) = serrecv;
    break;
  }
}

bool debounce(int pin, int *state, int *laststate, byte aimstate = HIGH)      // debounce button input
{
  bool result = false;
  int reading = digitalRead(pin);
  if (reading != *laststate) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != *state) {
      *state = reading;

      if (*state == aimstate)
        result = true;
    }
  }
  *laststate = reading;
  return result;
}

void saveConf()       // save conf in EEPROM
{
  EEPROM.put(0, settings);
  play_melody(false);
}

void loadConf()       // read conf from EEPROM
{
  for (int i=0; i<sizeof(settings.signature); i++)
  {
    if ((char)EEPROM.read(i) != settings.signature[i])
    {
      saveConf();   // if invalid signature, clear EEPROM
      break;
    }
  }
  EEPROM.get(0, settings);
}

void setup()                // setup() function to setup all necessary parameters before we go to endless loop() function
{
#if !defined(ARDUINO_SAMD_ZERO)
  makePinsInput();
  EICRA = 0x00;                  //configure INT0 to trigger on low level
#endif // !defined(ARDUINO_SAMD_ZERO)
  Serial.begin(UART_SPEED);       // set up arduino serial port
  Wire.begin();             // lets init i2c protocol
  while(!ms5611.begin())
  {
    //Serial.println("Could not find a valid MS5611 sensor, check wiring!");
    for (int i=0; i<10; i++)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(150);
      digitalWrite(LED_BUILTIN, LOW);
      delay(150);
    }
  }

  loadConf();         //load conf from EEPROM
  play_melody();      //everything is ready, play "welcome" sound
}

void loop(void)
{
  tempo = millis();
  vario = 0;
  N1 = 0;
  N2 = 0;
  N3 = 0;
  D1 = 0;
  D2 = 0;

  Altitude = ms5611.getAltitude(ms5611.readPressure(true), settings.p0); //take new altitude in meters
  Pressure = getPressure(Altitude, P0); // get back pressure possibly compensated with settings.p0
  average_pressure = Averaging_Filter(k, (unsigned long)Pressure); // take average (used for LK8000)

  for(int cc = 1; cc <= maxsamples; cc++)                              //samples averaging and vario algorithm
  {
    alt[(cc - 1)] = alt[cc];
    tim[(cc - 1)] = tim[cc];
  };
  alt[maxsamples] = Altitude;
  tim[maxsamples] = tempo;
  float stime = tim[maxsamples - samples];
  for(int cc = (maxsamples - samples); cc < maxsamples; cc++)
  {
    N1 += (tim[cc] - stime) * alt[cc];
    N2 += (tim[cc] - stime);
    N3 += (alt[cc]);
    D1 += (tim[cc] - stime) * (tim[cc] - stime);
    D2 += (tim[cc] - stime);
  };

  vario = 1000 * ((samples * N1) - N2 * N3) / (samples * D1 - D2 * D2);
  if ((tempo - beep) > Beep_period)                      // make some beep
  {
    beep = tempo;

    if ((vario < 0 ) && (thermalling == true))     //looks like we jump out the thermall
    {
      //Beep_period=1000;
      //tone(speaker_pin,50, 500); //oo, we lost thermall play alarm
      thermalling = false;
    }
    else if ((vario > vario_climb_rate_start || vario < vario_sink_rate_start) && vario < 15 )
    {
      thermalling = (vario > 0);
      Beep_period = min(1000, 350 - (vario * 5));
      tone(speaker_pin, max(100, (1000 + (100 * vario))), min(1000, 300 - (vario * 5))); //when climbing make faster and shorter beeps
    }
  }

  if (millis() >= get_time2)    //every second get temperature and battery level
  {
    Temperature = ms5611.readTemperature(); // get temperature in celsius from time to time, we have to divide that by 10 to get XY.Z
    my_temperature = Temperature;
    Battery_Vcc = min(1100, 1000 + max(0, Averaging_Filter(battper, (readVcc() - 3.6f) / 0.006f))); // get voltage and prepare in percentage (3.6V => 0%, 4.2V => 100%)
    get_time2 = millis() + PERIOD_BAT;
#ifdef BLINK_LED
    // blink the led
    light_on = !light_on;
    digitalWrite(LED_BUILTIN, light_on ? HIGH : LOW);
#endif
  }

  if (millis() >= get_time3)     //every 1/3 second send NMEA output over serial port
  {
    String str_out;
    if (settings.type_sentences == TYPE_BT_SENTENCES::LXNAV)
    {
      //creating now NMEA serial output for LXNAV. LXWP0 sentence format:
      //$LXWP0,logger_stored, airspeed, airaltitude,v1[0],v1[1],v1[2],v1[3],v1[4],v1[5], hdg, windspeed*CS<CR><LF>
      // 0 loger_stored : [Y|N] (not used in LX1600)
      // 1 IAS [km/h] ----> Condor uses TAS!
      // 2 baroaltitude [m]
      // 3-8 vario values [m/s] (last 6 measurements in last second)
      // 9 heading of plane (not used in LX1600)
      // 10 windcourse [deg] (not used in LX1600)
      // 11 windspeed [km/h] (not used in LX1600)
      // $LXWP0,Y,222.3,1665.5,1.71,,,,,,239,174,10.1
      str_out = String("LXWP0" + String(",Y,,") + String(dtostrf(Altitude, 0, 0, altitude_arr)) + String(",")
       + String(dtostrf((vario), 0, 3, vario_arr)) + String(",,,,,,,,"));
    }
    else
    {
      //creating now NMEA serial output for LK8000. LK8EX1 protocol format:
      //$LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
      str_out = String("LK8EX1" + String(",") + String(average_pressure, DEC) + String(",") + String(dtostrf(Altitude, 0, 0, altitude_arr)) + String(",") +
                String(dtostrf((vario * 100), 0, 0, vario_arr)) + String(",") + String(my_temperature, DEC) + String(",") + String(Battery_Vcc, DEC) + String(","));
    }
    unsigned int checksum_end, ai, bi;                                               // Calculating checksum for data string
    for (checksum_end = 0, ai = 0; ai < str_out.length(); ai++)
    {
      bi = (unsigned char)str_out[ai];
      checksum_end ^= bi;
    }
    Serial.print("$");                     //print first sign of NMEA protocol
    Serial.print(str_out);                 // print data string
    Serial.print("*");                     //end of protocol string
    Serial.println(checksum_end, HEX);     //print calculated checksum on the end of the string in HEX
    get_time3 = millis() + PERIOD_NMEA;
  }

  if (Serial.available())
    ParseCommand();          //parse commands from user

#if !defined(ARDUINO_SAMD_ZERO)
  // is button is released, power off the µc
  if (debounce(button_pin, &buttonState, &lastButtonState, HIGH))
  {
    if (awakening)
      awakening = false;
    else
      goToSleep();
  }
#endif // !defined(ARDUINO_SAMD_ZERO)
}
//The End
