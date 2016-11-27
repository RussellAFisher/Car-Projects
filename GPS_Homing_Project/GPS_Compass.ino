#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "Adafruit_NeoPixel.h"

SoftwareSerial mySerial(3, 2);

Adafruit_GPS GPS(&mySerial);

//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2

#define GPSECHO false

//--------------------------------------------------|
//                    WAYPOINTS                     |
//--------------------------------------------------|
////Galvanize                             |
  #define GEO_LAT                39.757784
  #define GEO_LON                -105.007224
  
//--------------------------------------------------|

//--------------------------------------------------|
//                    DISTANCE                      |
//--------------------------------------------------|
//Please enter the distance (in meters) from your   |
//destination that you want your LEDs to light up green:  |
  #define DESTINATION_DISTANCE   50
//--------------------------------------------------|


// Navigation location
float targetLat = GEO_LAT;
float targetLon = GEO_LON;

float distanceTil = 0;
float counter=0;

// Trip distance
float tripDistance;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(12, 6, NEO_RGB + NEO_KHZ800);

long distanceIntensity;

boolean usingInterrupt = false;
void useInterrupt(boolean);

void setup()  
{
  Serial.begin(4800);

  GPS.begin(9600);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  
 
  GPS.sendCommand(PGCMD_ANTENNA); 
  
  useInterrupt(true);

  delay(1000);
  
  Serial.println(PMTK_Q_RELEASE);
  
  strip.begin();

  strip.show();
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

void loop()
{
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
      if (c) Serial.print(c);
  
  if (GPS.newNMEAreceived()) {
  
    if (!GPS.parse(GPS.lastNMEA()))
      return; 
  }
  
  if (timer > millis())  timer = millis();
  
  if (millis() - timer > 2000) { 
    timer = millis(); 
  
  if (GPS.fix) {
        
         Serial.print("\nLocation: ");
         Serial.print(GPS.latitudeDegrees, 5);
         Serial.print(", "); 
         Serial.println(GPS.longitudeDegrees, 5);
         Serial.print("Destination: ");
         Serial.print(targetLat, 5);
         Serial.print(", "); 
         Serial.println(targetLon, 5);
         Serial.print("Distance to: ");
         Serial.println(distanceTil);
         Serial.print("Speed: ");
         Serial.println(GPS.speed);
         delay(1000);
        
         float fLat = decimalDegrees(GPS.latitude, GPS.lat);
         float fLon = decimalDegrees(GPS.longitude, GPS.lon);
         tripDistance = (double)calc_dist(fLat, fLon, targetLat, targetLon);
         
         headingDistance((double)calc_dist(fLat, fLon, targetLat, targetLon));
        
         if ((calc_bearing(fLat, fLon, targetLat, targetLon) - GPS.angle) > 0) {
              headingDirection(calc_bearing(fLat, fLon, targetLat, targetLon)-GPS.angle);
          }
         else {
              headingDirection(calc_bearing(fLat, fLon, targetLat, targetLon)-GPS.angle+360);
          } 
 
         Serial.print("Distance Remaining: "); Serial.println((double)calc_dist(fLat, fLon, targetLat, targetLon));
         distanceTil = ((double)calc_dist(fLat, fLon, targetLat, targetLon));
    }
  }
}

int calc_bearing(float flat1, float flon1, float flat2, float flon2)
{
  float calc;
  float bear_calc;

  float x = 69.1 * (flat2 - flat1); 
  float y = 69.1 * (flon2 - flon1) * cos(flat1/57.3);

  calc=atan2(y,x);

  bear_calc= degrees(calc);

  if(bear_calc<=1){
    bear_calc=360+bear_calc; 
  }
  return bear_calc;
}

void headingDirection(float heading) 
{
  if ((heading >= 345)||(heading < 15)) {
    Serial.println("  Top");
    for (uint16_t i=0;i<strip.numPixels();i++) {
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
    strip.setPixelColor(0, strip.Color(distanceIntensity,0,0));
  }
  
  else if ((heading >= 15)&&(heading < 45)) {
    Serial.println(" Rightish");
    for (uint16_t i=0;i<strip.numPixels();i++) {
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
    strip.setPixelColor(1, strip.Color(distanceIntensity,(distanceIntensity*.25),0));
  }
  
  else if ((heading >= 45)&&(heading < 75)) {
    Serial.println("  Rightish");
    for (uint16_t i=0;i<strip.numPixels();i++) {
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
    strip.setPixelColor(2, strip.Color(distanceIntensity,(distanceIntensity*.75),0));
  }
  
  else if ((heading >= 75)&&(heading < 105)) {
    Serial.println(" Right");
    for (uint16_t i=0;i<strip.numPixels();i++) {
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
    strip.setPixelColor(3, strip.Color(distanceIntensity,distanceIntensity,0));
  }
  
  else if ((heading >= 105)&&(heading < 135)) {
    Serial.println("  Back right");
    for (uint16_t i=0;i<strip.numPixels();i++) {
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
    strip.setPixelColor(4, strip.Color((distanceIntensity*.75),distanceIntensity,0));
  }
  
  else if ((heading >= 135)&&(heading < 165)) {
    Serial.println(" Back right");
    for (uint16_t i=0;i<strip.numPixels();i++) {
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
    strip.setPixelColor(5, strip.Color((distanceIntensity*.25),distanceIntensity,0));
  }
  
  else if ((heading >= 165)&&(heading < 195)) {
    Serial.println("  Turn around");
    for (uint16_t i=0;i<strip.numPixels();i++) {
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
    strip.setPixelColor(6, strip.Color(0,distanceIntensity,0));
  }
  
  else if ((heading >= 195)&&(heading < 225)) {
    Serial.println(" Back left");
    for (uint16_t i=0;i<strip.numPixels();i++) {
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
    strip.setPixelColor(7, strip.Color((distanceIntensity*.25),distanceIntensity,0));
  }
  
  else if ((heading >= 225)&&(heading < 255)) {
    Serial.println(" Back left");
    for (uint16_t i=0;i<strip.numPixels();i++) {
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
    strip.setPixelColor(8, strip.Color((distanceIntensity*.75),distanceIntensity,0));
  }
  
  else if ((heading >= 255)&&(heading < 285)) {
    Serial.println(" Left");
    for (uint16_t i=0;i<strip.numPixels();i++) {
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
    strip.setPixelColor(9, strip.Color(distanceIntensity,distanceIntensity,0));
  }
  
  else if ((heading >= 285)&&(heading < 315)) {
    Serial.println(" Leftish");
    for (uint16_t i=0;i<strip.numPixels();i++) {
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
    strip.setPixelColor(10, strip.Color(distanceIntensity,(distanceIntensity*.75),0));
  }
  
  else if ((heading >= 315)&&(heading < 345)) {
    Serial.println(" Leftish");
    for (uint16_t i=0;i<strip.numPixels();i++) {
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
    strip.setPixelColor(11, strip.Color(distanceIntensity,(distanceIntensity*.25),0));
  }
  strip.show();
}

void headingDistance(float fDist)
{
 Serial.println(fDist);
 Serial.print("Calculating distance");

  if ((fDist < DESTINATION_DISTANCE)) { 
    Serial.println("Arrived at destination!");
      distanceIntensity=25;
  }
  
  if ((fDist > DESTINATION_DISTANCE && fDist <= 250)) {
      distanceIntensity=20;
  }
  
  if ((fDist > 250 && fDist <= 1000)) {
      distanceIntensity=15;
  }
  
  if ((fDist > 1000 && fDist <= 5000)) {
      distanceIntensity=10;
  }
  
  if ((fDist > 5000)) {
      distanceIntensity=5;
  }
}

unsigned long calc_dist(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  diflat=radians(flat2-flat1);
  flat1=radians(flat1);
  flat2=radians(flat2);
  diflon=radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

  dist_calc*=6371000.0; //Converting to meters
  return dist_calc;
}

// Convert NMEA coordinate to decimal degrees
float decimalDegrees(float nmeaCoord, char dir) {
  uint16_t wholeDegrees = 0.01*nmeaCoord;
  int modifier = 1;

  if (dir == 'W' || dir == 'S') {
    modifier = -1;
  }
  
  return (wholeDegrees + (nmeaCoord - 100.0*wholeDegrees)/60.0) * modifier;
}
