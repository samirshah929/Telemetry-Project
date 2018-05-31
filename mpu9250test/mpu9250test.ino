/*
 * Junior Design Final Project, Telemetry System
 * MPU9250 polling test function, using MPU9250 library by Border Flight Systems
 * Samir Shah
*/
#include "MPU9250.h"
#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps_tiny;
SoftwareSerial gps(7, 8);

static const int RXPin = 0, TXPin = 1; // if this shit doenst work, switch them
static const uint32_t xbeeBaud = 9600;

static void smartdelay(unsigned long ms);
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);



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
//SoftwareSerial ss(RXPin, TXPin);

MPU9250 IMU(SPI,10);
int status;


void setup() {
  Serial.begin(9600);
  //ss.begin(xbeeBaud);
  gps.begin(9600);
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
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
  gps_tiny.f_get_position(&flat, &flon, &age);

//  Serial.print(flat, 10);
//  Serial.print("\t");
//  Serial.println(flon, 10);

  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);

//  SEND(flat, 8);
//  SEND(flon, 9);

  Serial.print(flat);
  Serial.print(flon);
  
  
  
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

//  SEND(IMU.getAccelX_mss(), 0);
//  SEND(IMU.getAccelY_mss(), 1);
//  SEND(IMU.getAccelZ_mss(), 2);  
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
//  SEND(IMU.getTemperature_C(), 13);
  
  delay(1000);
  Serial.println();
}

//void SEND(float val, int id) {
//
//  char strt = '$';
//  char stp = '#';
//
//  byte *ID = (byte*) &id;
//  byte *NUM = (byte*) &val;
//
//  Serial.write(ss.write('$'));
//  Serial.write(ss.write(ID,2));
//  Serial.write(ss.write('#'));
//  Serial.write(ss.write('$'));
//  Serial.write(ss.write(NUM,4));
//  Serial.write(ss.write('#'));
//}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gps.available())
      gps_tiny.encode(gps.read());
  } while (millis() - start < ms);
}

static void print_float(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartdelay(0);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartdelay(0);
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print("********** ******** ");
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
        month, day, year, hour, minute, second);
    Serial.print(sz);
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay(0);
}

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartdelay(0);
}
