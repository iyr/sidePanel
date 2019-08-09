#include "FastLED.h"
#include <TimeLib.h>
#include <PacketSerial.h>

#define UPPER_LEDS  25
#define LOWER_LEDS  25
#define RIGHT_LEDS  16
#define LEFT_LEDS   16
#define FRONT_LEDS  32
#define PANEL_LEDS  13

#define UPPER_PIN 30
#define LEFT_PIN  29
#define LOWER_PIN 28
#define RIGHT_PIN 27
#define FRONT_PIN 25
#define PANEL_PIN 33

CRGB frontBar[FRONT_LEDS];
CRGB upperBar[UPPER_LEDS];
CRGB lowerBar[LOWER_LEDS];
CRGB rightBar[RIGHT_LEDS];
CRGB leftBar [LEFT_LEDS ];
CRGB panelLeds[PANEL_LEDS];

/*
   Serial Port Layout:
   0: Debug/Development, native Teensy USB header
   1: Adafruit Metro 328, for comms with keyboard
   2: Adafruit FTDI breakout configure as USB/serial, for comms with PC
   3: N/C
   4: Raspberry Pi
*/
PacketSerial aMetro;
PacketSerial RaspPi;

int     frameCursor = 0;
int     b           = 2;
int     pos         = 0;
int     start       = 0;
int     finish      = 0;
int     timeScale   = 0;
uint8_t curHour     = 0;
uint8_t curMin      = 0;
uint8_t curTen      = 0;
uint8_t animation   = 0;
uint8_t toMetro[8]  = {0};
uint8_t toRaspi[8]  = {0};

void setup() {
  setSyncProvider(getTeensy3Time);

  Serial1.begin(38400);
  aMetro.begin(&Serial1);
  aMetro.setPacketHandler(&onMetroPacket);

  Serial4.begin(115200);
  RaspPi.begin(&Serial4);
  RaspPi.setPacketHandler(&onRaspPiPacket);

  Serial.begin(112500);
  //  while (!Serial);  // Wait for Arduino Serial Monitor to open
  delay(100);
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }
  FastLED.addLeds<NEOPIXEL, FRONT_PIN>(frontBar, FRONT_LEDS);
  FastLED.addLeds<NEOPIXEL, UPPER_PIN>(upperBar, UPPER_LEDS);
  FastLED.addLeds<NEOPIXEL, LOWER_PIN>(lowerBar, LOWER_LEDS);
  FastLED.addLeds<NEOPIXEL, RIGHT_PIN>(rightBar, RIGHT_LEDS);
  FastLED.addLeds<NEOPIXEL, LEFT_PIN> (leftBar,  LEFT_LEDS  );
  FastLED.addLeds<NEOPIXEL, PANEL_PIN>(panelLeds, PANEL_LEDS);
  FastLED.setBrightness(255);
  frameCursor = 1;
}

void loop() {
  int rB = constrain(8    + map(pos, 0, 255, 0, 64), 0, 255);
  int gB = constrain(24   + map(pos, 0, 255, 0, -43), 0, 255);
  int bB = constrain(16   + map(pos, 0, 255, 0, 128), 0, 255);
  int rP = constrain(255  + map(pos, 0, 255, 0, 64), 0, 255);
  int gP = constrain(109  + map(pos, 0, 255, 0, -43), 0, 255);
  int bP = constrain(16   + map(pos, 0, 255, 0, 128), 0, 255);

  if (Serial.available() > 0) {
    time_t t = processSyncMessage();
    if (t != 0) {
      Teensy3Clock.set(t); // set the RTC
      setTime(t);
    }
  }

  if (frameCursor % 128 == 0) {
    curMin  = getMinuteOne();
    curTen  = getMinuteTen();
    curHour = hour();
    aMetro.send(toMetro, 8);
  }

  toMetro[0]  = curMin;
  toMetro[1]  = curTen;
  toMetro[2]  = curHour;

  if (frameCursor >= 512) {
    frameCursor = 0;
  } else if (frameCursor < 0) {
    frameCursor = 0;
  } else {
    frameCursor++;
    if (animation == 0) delay(timeScale);
  }

  if (animation == 0) {
    pulse(frontBar, FRONT_LEDS, 0, b, rB, gB, bB, rP, gP, bP);
    fill_solid(upperBar, UPPER_LEDS, CRGB(255, 255, 255));
    fill_solid(lowerBar, LOWER_LEDS, CRGB(255, 255, 255));
    fill_solid(rightBar, RIGHT_LEDS, CRGB(255, 255, 255));
    fill_solid(leftBar, LEFT_LEDS, CRGB(255, 255, 255));
    fill_solid(panelLeds, PANEL_LEDS, CRGB(192, 64, 255));
    //    pulse(upperBar, UPPER_LEDS, 0, b, rB, gB, bB, rP, gP, bP);
    //    pulse(lowerBar, LOWER_LEDS, 0, b, rB, gB, bB, rP, gP, bP);
    //    pulse(rightBar, RIGHT_LEDS, 0, b, rB, gB, bB, rP, gP, bP);
    //    pulse(leftBar,  LEFT_LEDS , 0, b, rB, gB, bB, rP, gP, bP);
  }
  else if (animation == 1)
  {
    confetti(frontBar,  FRONT_LEDS,   0);
    fill_solid(upperBar,  UPPER_LEDS, CHSV(pos, 192, 255));
    fill_solid(lowerBar,  LOWER_LEDS, CHSV(pos, 192, 255));
    fill_solid(rightBar,  RIGHT_LEDS, CHSV(pos, 192, 255));
    fill_solid(leftBar,   LEFT_LEDS,  CHSV(pos, 192, 255));
    fill_solid(panelLeds, PANEL_LEDS, CHSV(pos, 255, 255));
    //    confetti(upperBar,  UPPER_LEDS,   0);
    //    confetti(lowerBar,  LOWER_LEDS,   0);
    //    confetti(rightBar,  RIGHT_LEDS,   0);
    //    confetti(leftBar,   LEFT_LEDS,    0);
    //    confetti(panelLeds, PANEL_LEDS,   0);
    delay(5);
  }

  FastLED.show();
  //RaspPi.send(toRaspi, 8);
  aMetro.update();
  //RaspPi.update();
}

void onRaspPiPacket(const uint8_t* buffer, size_t size)
{
  uint8_t tmp[size];
  memcpy(tmp, buffer, size);
}

void onMetroPacket(const uint8_t* buffer, size_t size)
{
  uint8_t tmp[size];
  memcpy(tmp, buffer, size);
  frameCursor = tmp[0];
  pos         = tmp[1];
  animation   = tmp[2];
  finish      = millis();
  timeScale   = constrain((finish - start) / 512, 0, 500);
  start       = millis();
}

void confetti(CRGB * strip, int num_frontBar, int offset_from_start) {
  //Serial.println("Calling confetti");
  int cursorFast = random16(offset_from_start, num_frontBar);
  fadeToBlackBy(strip, num_frontBar, 2);
  strip[cursorFast] += CHSV(pos + random(-32, 32), 255, 128);
  strip[num_frontBar - cursorFast] += CHSV(pos + random(-32, 32), 255, 128);
}

void pulse( CRGB* strip,
            int   num_frontBar,
            int   offset_from_start,
            int   behavior,
            int   rBase,
            int   gBase,
            int   bBase,
            int   rPulse,
            int   gPulse,
            int   bPulse
          )
{
  /*
     Description of behaviors:
     0: Out-In
     1: Downstream
     2: Mid-Out
     3: Upstream
  */
  num_frontBar = num_frontBar - offset_from_start;

  int rComp   = 0;
  int gComp   = 0;
  int bComp   = 0;

  int pixel_offset = int(512.0 / float(num_frontBar));
  int upper_limit = (behavior % 2 == 0 ? (num_frontBar / 2) : num_frontBar);

  for (int i = 0; i < upper_limit; i++) {
    int c = constrain(255 - abs(2 * frameCursor - (255 + i * pixel_offset) + 1), 0, 255);
    //    if(i == 0)
    //      Serial.println(c);
    rComp = (rBase == rPulse ? rBase : map(c, 0, 255, rBase, rPulse));
    gComp = (gBase == gPulse ? gBase : map(c, 0, 255, gBase, gPulse));
    bComp = (bBase == bPulse ? bBase : map(c, 0, 255, bBase, bPulse));

    switch (behavior)
    {
      // "Out-in" loop:
      case 0:
        {
          strip[offset_from_start + i] = CRGB(rComp, gComp, bComp);
          strip[offset_from_start + num_frontBar - 1 - i] = CRGB(rComp, gComp, bComp);
        }
        break;

      // "Mid-out" loop:
      case 2:
        {
          strip[offset_from_start + num_frontBar / 2 - i - 1] = CRGB(rComp, gComp, bComp);
          strip[offset_from_start + num_frontBar / 2 + i] = CRGB(rComp, gComp, bComp);
        }
        break;

      // "Upstream" loop:
      case 1:
        strip[offset_from_start + i] = CRGB(rComp, gComp, bComp);
        break;

      // "Downstream" loop:
      case 3:
        strip[offset_from_start + num_frontBar - 1 - i] = CRGB(rComp, gComp, bComp);
        break;
    }
  }
}

int getMinuteOne() {
  int       curMin  =   minute();
  if (curMin < 10) return curMin;
  else
  {
    while (curMin >= 10) curMin -= 10;
    return curMin;
  }
}

int getMinuteTen() {
  int       curMin  =   minute() - getMinuteOne();
  switch (curMin)
  {
    case 0:
      return 0;

    case 10:
      return 1;

    case 20:
      return 2;

    case 30:
      return 3;

    case 40:
      return 4;

    case 50:
      return 5;
  }
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

/*  code to process time sync messages from the serial port   */
#define TIME_HEADER  "T"   // Header tag for serial time sync message

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    return pctime;
    if ( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
      pctime = 0L; // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}
