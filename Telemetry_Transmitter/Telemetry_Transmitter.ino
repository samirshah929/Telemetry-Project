#include <SoftwareSerial.h>
#include "MPU9250.h" //library for MPU9250 9-axis IMU (accelerometer, gyroscope, magnetometer)
#include <SPI.h>  //REMOVE IF ALREADY DEFINED ERROR COMES UP (might be included already by another library)
#include <SD.h>
#include <TinyGPS.h>
#include <MadgwickAHRS.h>  //make sure library is installed in arduino IDE (go to install library and search for madgwick)

#include <Wire.h> // i2c
#include <MS5611.h> // library for MS5611 (barometer) module (i2c)
/*
 * Wire Library
 */
//FLAGS FOR WHAT TO SEND/RECORD (1 = SEND/RECORD, 0 = DON'T SEND/RECORD)
#define GPS_SEND 0
#define ACCEL_SEND 0
#define GYRO_SEND 0
#define MAG_SEND 0
#define ORIENTATION_SEND 0
#define ALTITUDE_SEND 0
#define TEMPERATURE_SEND 0

#define GPS_RECORD 0
#define ACCEL_RECORD 0
#define GYRO_RECORD 0
#define MAG_RECORD 0
#define ORIENTATION_RECORD 0
#define ALTITUDE_RECORD 0
#define TEMPERATURE_RECORD 0
/******GPS******/
static const int GPSRxPin = 7, GPSTxPin = 8; // if this shit doenst work, switch them
static const uint32_t GPSBaud = 9600;

/******Sensor Block******/
static const int XbeeRxPin = 0, XbeeTxPin = 1; // if this shit doenst work, switch them
static const uint32_t xbeeBaud = 9600;

/******Barometer******/

/******Xbee Pro S3B******/



TinyGPS gps;
SoftwareSerial gps_mod(GPSRxPin, GPSTxPin);
SoftwareSerial xbee(XbeeRxPin, XbeeTxPin);

/*
 * GPS Functions
 */

static void smartdelay(unsigned long ms);
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);

/*
 * Sensors Functions
 */

MPU9250 IMU(SPI,10);
int status;
Madgwick filter;


void setup()
{
/*  Serial Begin  */

  Serial.begin(9600);
  xbee.begin(xbeeBaud);
  gps_mod.begin(GPSBaud);

/*  IMU Operation  */

  // Set up for IMU and Madgwick AHRS filter
  status = IMU.begin();
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);  //Gyro range set to +/- 250 dps
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);    //Accel range set to +/- 2g
  filter.begin(25);   //Madgwick filter initialiazed
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

}

void loop()
{
  // read the sensor
  IMU.readSensor();

  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;


  gps.f_get_position(&flat, &flon, &age);

    Serial.print("Lat:");
    Serial.print("\t");
    Serial.println(flat, 6);

    Serial.print("Lon:");
    Serial.print("\t");
    Serial.println(flon, 6);
    Serial.println();
    SEND(flat, 8);
    SEND(flon, 9);

  smartdelay(1000); //part of gps functionality
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (gps_mod.available())
      gps.encode(gps_mod.read());
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
void SEND(float val, int id) {

  char strt = '$';
  char stp = '#';

  byte *ID = (byte*) &id;
  byte *NUM = (byte*) &val;

  Serial.write(xbee.write('$'));
  Serial.write(xbee.write(ID,2));
  Serial.write(xbee.write('#'));
  Serial.write(xbee.write('$'));
  Serial.write(xbee.write(NUM,4));
  Serial.write(xbee.write('#'));
}

static void gps_print_loop(int loops)
{
  gps_print_header();
  for(int i = 0; i < 10; i++){
    gps_print();
  }
}

static void gps_print()
{
  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

   print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
   print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
    gps.f_get_position(&flat, &flon, &age);
   print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
   print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
   print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
   print_date(gps);
   print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 7, 2);
   print_float(gps.f_course(), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
   print_float(gps.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, 6, 2);
   print_str(gps.f_course() == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(gps.f_course()), 6);
   print_int(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0xFFFFFFFF : (unsigned long)TinyGPS::distance_between(flat, flon, LONDON_LAT, LONDON_LON) / 1000, 0xFFFFFFFF, 9);
   print_float(flat == TinyGPS::GPS_INVALID_F_ANGLE ? TinyGPS::GPS_INVALID_F_ANGLE : TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
   print_str(flat == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON)), 6);

   gps.stats(&chars, &sentences, &failed);
   print_int(chars, 0xFFFFFFFF, 6);
   print_int(sentences, 0xFFFFFFFF, 10);
   print_int(failed, 0xFFFFFFFF, 9);
   Serial.println();

    smartdelay(1000);
}
static void gps_print_header()
{
  Serial.print("Testing TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();
  Serial.println("Sats HDOP Latitude  Longitude  Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum");
  Serial.println("          (deg)     (deg)      Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail");
  Serial.println("-------------------------------------------------------------------------------------------------------------------------------------");
}
