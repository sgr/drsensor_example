#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "Ultrasonic.h"
#include <Wire.h>
#include <ADXL345.h>
#include <stdio.h>

// 時刻のキャッシュ
long lastMillis = millis();
int year = -1;
int month = -1;
int day = -1;
int hour = -1;
int minute = -1;
int second = -1;

SoftwareSerial ss(2, 3); //GPSはD2に接続
TinyGPSPlus gps; // The TinyGPS++ object
Ultrasonic ultrasonic(8); //超音波センサーはD8に接続
ADXL345 adxl; //variable adxl is an instance of the ADXL345 library

void setup()
{
  ss.begin(9600);
  ss.listen();
  Serial.begin(9600);
  setupAdxl();
}

void loop()
{
  smartDelay(1000); // GPSのデータを取得
  String gpsLine = String(gpstime() + "," + gpsloc());

  // 超音波センサーより前方障害物との距離の取得 (cm)
  char distance[10];
  long rangeInCentimeters = ultrasonic.MeasureInCentimeters();
  sprintf(distance, "%ld", rangeInCentimeters);
 
  // 加速度計の値取得
  char accel[30];
  double xyz[3];
  char x[10];
  char y[10];
  char z[10];
  adxl.getAcceleration(xyz);
  sprintf(accel, "%s,%s,%s", dtostrf(xyz[0], 7, 3, x), dtostrf(xyz[1], 7, 3, y), dtostrf(xyz[2], 7, 3, z));

  // 一行出力
  String output = String(gpsLine + ", " + String(distance) + "," + String(accel));
  Serial.println(output);
  Serial.flush();
}

void setupAdxl()
{
  adxl.powerOn();

  //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(75); //62.5mg per increment
  adxl.setInactivityThreshold(75); //62.5mg per increment
  adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
 
  //look of activity movement on this axes - 1 == on; 0 == off 
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);
 
  //look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(1);
 
  //look of tap movement on this axes - 1 == on; 0 == off
  adxl.setTapDetectionOnX(0);
  adxl.setTapDetectionOnY(0);
  adxl.setTapDetectionOnZ(1);
 
  //set values for what is a tap, and what is a double tap (0-255)
  adxl.setTapThreshold(50); //62.5mg per increment
  adxl.setTapDuration(15); //625us per increment
  adxl.setDoubleTapLatency(80); //1.25ms per increment
  adxl.setDoubleTapWindow(200); //1.25ms per increment
 
  //set values for what is considered freefall (0-255)
  adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment
 
  //setting all interrupts to take place on int pin 1
  //I had issues with int pin 2, was unable to reset it
  adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
 
  //register interrupt actions - 1 == on; 0 == off  
  adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
}

String gpstime()
{
  char datetime[24];
  if (gps.date.isValid() && gps.time.isValid()) {
    // 大域変数に最後に取得できた時刻をキャッシュする
    lastMillis = millis();
    year = gps.date.year();
    month = gps.date.month();
    day = gps.date.day();
    hour = gps.time.hour();
    minute = gps.time.minute();
    second = gps.time.second();
    sprintf(datetime, "%04d%02d%02d%02d%02d%02d", gps.date.year(), gps.date.month(), gps.date.day(),  gps.time.hour(), gps.time.minute(), gps.time.second());
  } else if (year > 0 && month > 0 && day > 0 && hour > 0 && minute > 0 && second > 0) {
    long diff = (millis() - lastMillis) * 1000;
    long nsec = second + diff;
    long nmin = minute + (nsec / 60);
    long nhour = hour + (nmin / 60);
    sprintf(datetime, "%04d%02d%02d%02d%02d%02d", year, month, day, hour, nmin % 60, nsec % 60);
  } else {
    sprintf(datetime, "NODATE");
  }
  return String(datetime);
}

String gpsloc()
{
  char loc[26];
  char lat[12];
  char lng[12];
  if (gps.location.isValid()) {
    sprintf(loc, "%s, %s", dtostrf(gps.location.lat(), 10, 6, lat), dtostrf(gps.location.lng(), 10, 6, lng));
  } else {
    sprintf(loc, "NOLAT, NOLNG");
  }
  return String(loc);
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do {
    while (ss.available()) {
      gps.encode(ss.read());
    }
  } while (millis() - start < ms);
}

