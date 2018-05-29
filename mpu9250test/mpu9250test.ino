/*
 * Junior Design Final Project, Telemetry System
 * MPU9250 polling test function, using MPU9250 library by Border Flight Systems
 * Samir Shah
*/
#include "MPU9250.h"
#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps_tiny;
SoftwareSerial gps(8, 7);

static const int RXPin = 1, TXPin = 0; // if this shit doenst work, switch them
static const uint32_t xbeeBaud = 9600;

/*
Pins for teensy:
SDA -> Pin 11 (MOSI)
AD0 -> Pin 12 (MISO)
SCL -> Pin 13 (SCK)
NCS -> Pin 10 (SS)
VCC -> 3.3V
GND -> GND
*/
// an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10
SoftwareSerial ss(RXPin, TXPin);

MPU9250 IMU(SPI,10);
int status;
void setup() {
  Serial.begin(9600);
  ss.begin(xbeeBaud);
  gps.begin(4800);
  // start communication with IMU 
  status = IMU.begin();
  /*
  if (status < 0) {
    xBee.println("IMU initialization unsuccessful");
    xBee.println("Check IMU wiring or try cycling power");
    xBee.print("Status: ");
    xBee.println(status);
    while(1) {}
  }*/
}
void loop() {
  // read the sensor
  IMU.readSensor();
  // display the data
  float flat, flon;
  unsigned long age;
  gps_tiny.f_get_position(&flat, &flon, &age);

  Serial.print(flat, 6);
  Serial.print("\t");
  Serial.println(flon, 6);
  
  
  
  /*Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.print("\t");
  Serial.println(IMU.getTemperature_C(),6);*/

  SEND(IMU.getAccelX_mss(), 0);
  SEND(IMU.getAccelY_mss(), 1);
  SEND(IMU.getAccelZ_mss(), 2);  
  /*
  delay(50);
  SEND(IMU.getGyroX_rads(), 10);
  SEND(IMU.getGyroY_rads(), 11);
  SEND(IMU.getGyroZ_rads(), 12);
  delay(50);
  SEND(IMU.getMagX_uT(), 5);
  SEND(IMU.getMagY_uT(), 6);
  SEND(IMU.getMagZ_uT(), 7);
  delay(50);*/
  SEND(IMU.getTemperature_C(), 13);
  
  delay(300);
}

void SEND(float val, int id) {

  char strt = '$';
  char stp = '#';

  byte *ID = (byte*) &id;
  byte *NUM = (byte*) &val;

  Serial.write(ss.write('$'));
  Serial.write(ss.write(ID,2));
  Serial.write(ss.write('#'));
  Serial.write(ss.write('$'));
  Serial.write(ss.write(NUM,4));
  Serial.write(ss.write('#'));
}
