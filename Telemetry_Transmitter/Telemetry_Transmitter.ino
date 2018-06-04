#include <SoftwareSerial.h>
//#include <ArduinoSTL.h>
#include "MPU9250.h" //library for MPU9250 9-axis IMU
                     //(accelerometer, gyroscope, magnetometer)
#include <SPI.h>     //REMOVE IF ALREADY DEFINED ERROR COMES UP
                     //(might be included already by another library)
#include <SD.h>
#include <TinyGPS.h>
#include <MadgwickAHRS.h>  //make sure library is installed in arduino IDE
                           //(go to install library and search for madgwick)
#include <Wire.h>    // i2c
#include <MS5611.h>  // library for MS5611 (barometer) module (i2c)
/*
 * Wire Library
 */
//FLAGS FOR WHAT TO SEND/RECORD (1 = SEND/RECORD, 0 = DON'T SEND/RECORD)
#define GPS_SEND            0
#define ACCEL_SEND          0
#define GYRO_SEND           0
#define MAG_SEND            0
#define ORIENTATION_SEND    0
#define ALTITUDE_SEND       0
#define TEMPERATURE_SEND    0
#define BATTERY_SEND        0

#define GPS_RECORD          0
#define ACCEL_RECORD        0
#define GYRO_RECORD         0
#define MAG_RECORD          0
#define ORIENTATION_RECORD  0
#define ALTITUDE_RECORD     1
#define TEMPERATURE_RECORD  0
#define BATTERY_RECORD      0

#define DEBUG_MODE          1
/******GPS******/
static const int GPSRxPin = 8, GPSTxPin = 7; // if this shit doenst work, switch them
static const uint32_t GPSBaud = 9600;
TinyGPS gps;  //GPS Object
SoftwareSerial gps_mod(GPSRxPin, GPSTxPin);



/******Sensor Block******/
static const int Sensor_SelPin = 20;   //Sensor Block Select Pin

/******Barometer******/

// SDAO = pin 18
// SCLO = pin 19
//  static const int Barometer_SelPin = 21;   //Barometer Select Pin

/******SD Card Module******/
static const int SD_SelPin = 22;   //SD Card Module Select Pin

/******EXTERNAL CONNECTOR******/
static const int EXT_SelPin = 23;  //EXTERNAL CONNECTOR Select Pin

/******Battery Voltage******/
static const int ADC_Pin = 2;  //EXTERNAL CONNECTOR Select Pin
//NOTE: DO NOT TOUCH PIN 15 !!!!!!!!

/******Xbee Pro S3B******/
static const int XbeeRxPin = 1, XbeeTxPin = 0;
static const uint32_t xbeeBaud = 9600;
SoftwareSerial xbee(XbeeRxPin, XbeeTxPin);

/******UI BUTTON******/
const byte interruptPin = 23;

//--------------------------------------------------------------------------//
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
MS5611 ms5611;
double referencePressure;
MPU9250 IMU(SPI,Sensor_SelPin);
int status;
Madgwick filter;

/*
 * Global Variables
 */
 //GPS
float flat, flon;
unsigned long age, date, time, chars = 0;
unsigned short sentences = 0, failed = 0;
static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
//Sensors
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float pitch, roll, yaw;
float voltage;
float barom;
float temp;
float realPressure;
long AnalogsensorValue;


volatile byte state = LOW;
File dataFile;
//--------------------------------------------------------------------------//
//******************************MAIN Setup**********************************//
//--------------------------------------------------------------------------//
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
//    while(1) {}
  }
  Serial.println("Initialize MS5611 Sensor");
  Record_Header();
  Serial_Header();

  if(ALTITUDE_SEND == 1 || ALTITUDE_RECORD == 1){
    while(!ms5611.begin())
    {
      Serial.println("Could not find a valid MS5611 sensor, check wiring!");
      delay(500);
    }

  // Get reference pressure for relative altitude

    referencePressure = ms5611.readPressure();
}
  // Attach interrupt for Button UI
attachInterrupt(digitalPinToInterrupt(interruptPin), Button_Press, CHANGE);


}
//-------------------------------------------------------------------------//
//*****************************   MAIN LOOP   *****************************//
//-------------------------------------------------------------------------//
void loop()
{
  Data_Retrieve();

  if(GPS_SEND == 1 || ACCEL_SEND == 1 || GYRO_SEND == 1 || MAG_SEND == 1 || ORIENTATION_SEND == 1 || ALTITUDE_SEND == 1 || TEMPERATURE_SEND == 1 || BATTERY_SEND == 1){
    Send_Data();
  }

  if(GPS_RECORD == 1 || ACCEL_RECORD == 1 || GYRO_RECORD == 1 || MAG_RECORD == 1 || ORIENTATION_RECORD == 1 || ALTITUDE_RECORD == 1 || TEMPERATURE_RECORD == 1 || BATTERY_RECORD == 1){
    Record_Data();
  }
  if(DEBUG_MODE == 1){
    Serial_Data();
  }
    delay(200);
}

//-------------------------------------------------------------------------//
//*****************************   Functions   *****************************//
//-------------------------------------------------------------------------//

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

//-------------------------------------------------------------------------//
//*****************************   Data Send   *****************************//
//-------------------------------------------------------------------------//
/*
 *  Main LUT for Main Loop
 */
static void Send_Data()
{
  if(GPS_SEND == 1){
    SEND(flat,  8);
    SEND(flon,  9);
    delay(50);
  }
  if(ACCEL_SEND == 1){
    SEND(ax,  0);
    SEND(ay,  1);
    SEND(az,  2);
    delay(50);
  }
  if(GYRO_SEND == 1){
    SEND(gx,  10);
    SEND(gy,  11);
    SEND(gz,  12);
    delay(50);
  }
  if(MAG_SEND == 1){
    SEND(mx,  5);
    SEND(my,  6);
    SEND(mz,  7);
    delay(50);
  }
  if(ORIENTATION_SEND == 1){
    SEND(pitch, 14);
    SEND(roll,  15);
    SEND(yaw,   16);
    delay(50);
  }
  if(ALTITUDE_SEND == 1){
    SEND(barom, 3);
    delay(50);
  }
  if(TEMPERATURE_SEND == 1){
    SEND(temp, 13);
    delay(50);
  }
  if(BATTERY_SEND == 1){
    SEND(voltage, 4);
    delay(50);
  }

}
//-------------------------------------------------------------------------//
//*****************************   Data Record   ***************************//
//-------------------------------------------------------------------------//
static void Record_Data(){
dataFile = SD.open("myfile.txt", FILE_WRITE);
    if(GPS_RECORD == 1){
      dataFile.print(flat, 6);
      dataFile.print(",");
      dataFile.print(flon, 6);
      dataFile.print(",");
    }
    if(ACCEL_RECORD == 1){
      dataFile.print(ax, 6);
      dataFile.print(",");
      dataFile.print(ay, 6);
      dataFile.print(",");
      dataFile.print(az, 6);
      dataFile.print(",");
    }
    if(GYRO_RECORD == 1){
      dataFile.print(gx, 6);
      dataFile.print(",");
      dataFile.print(gy, 6);
      dataFile.print(",");
      dataFile.print(gz, 6);
      dataFile.print(",");
    }
    if(MAG_RECORD == 1){
      dataFile.print(mx, 6);
      dataFile.print(",");
      dataFile.print(my, 6);
      dataFile.print(",");
      dataFile.print(mz, 6);
      dataFile.print(",");
    }
    if(ORIENTATION_RECORD == 1){
      dataFile.print(pitch, 6);
      dataFile.print(",");
      dataFile.print(roll, 6);
      dataFile.print(",");
      dataFile.print(yaw, 6);
      dataFile.print(",");
    }
    if(ALTITUDE_RECORD == 1){
      dataFile.print(barom, 6);
      dataFile.print(",");
    }
    if(TEMPERATURE_RECORD == 1){
      dataFile.print(temp, 6);
      dataFile.print(",");
    }
    if(BATTERY_RECORD == 1){
      dataFile.print(voltage, 6);
      dataFile.print(",");
    }
    dataFile.println();
    dataFile.close();
  }
static void Record_Header(){
    dataFile = SD.open("myfile.txt", FILE_WRITE);

    if(GPS_RECORD == 1){
      dataFile.print("Lat:  ,Lon:  ,");
    }
    if(ACCEL_RECORD == 1){
      dataFile.print("Ax :  ,Ay :  ,Az :  ,");
    }
    if(GYRO_RECORD == 1){
      dataFile.print("Gx :  ,Gy :  ,Gz :  ,");
    }
    if(MAG_RECORD == 1){
      dataFile.print("Mx :  ,My :  ,Mz :  ,");
    }
    if(ORIENTATION_RECORD == 1){
      dataFile.print("Pitch:,Roll: ,Yaw:  ,");
    }
    if(ALTITUDE_RECORD == 1){
      dataFile.print("Barom:,");
    }
    if(TEMPERATURE_RECORD == 1){
      dataFile.print("Temp: ,");
    }
    if(BATTERY_RECORD == 1){
      dataFile.print("Volt: ,");
    }
    dataFile.println();
    dataFile.println("_______________________________________________________________________________________________________________________");
    dataFile.close();
  }
static void Serial_Data(){
    if(GPS_RECORD == 1){
    Serial.print(flat, 6);
    Serial.print(",");
    Serial.print(flon, 6);
    Serial.print(",");
    }
    if(ACCEL_RECORD == 1){
      Serial.print(ax, 6);
      Serial.print(",");
      Serial.print(ay, 6);
      Serial.print(",");
      Serial.print(az, 6);
      Serial.print(",");
    }
    if(GYRO_RECORD == 1){
      Serial.print(gx, 6);
      Serial.print(",");
      Serial.print(gy, 6);
      Serial.print(",");
      Serial.print(gz, 6);
      Serial.print(",");
    }
    if(MAG_RECORD == 1){
      Serial.print(gx, 6);
      Serial.print(",");
      Serial.print(gy, 6);
      Serial.print(",");
      Serial.print(gz, 6);
      Serial.print(",");
    }
    if(ORIENTATION_RECORD == 1){
      Serial.print(pitch, 6);
      Serial.print(",");
      Serial.print(roll, 6);
      Serial.print(",");
      Serial.print(yaw, 6);
      Serial.print(",");
    }
    if(ALTITUDE_RECORD == 1){
      Serial.print(barom, 6);
      Serial.print(",");
    }
    if(TEMPERATURE_RECORD == 1){
      Serial.print(temp, 6);
      Serial.print(",");
    }
    if(BATTERY_RECORD == 1){
      Serial.print(voltage, 6);
      // Serial.print(",");
    }
    Serial.println();
  }
static void Serial_Header(){

    if(GPS_RECORD == 1){
      Serial.print("Lat:  ,Lon:  ,");
    }
    if(ACCEL_RECORD == 1){
      Serial.print("Ax :  ,Ay :  ,Az :  ,");
    }
    if(GYRO_RECORD == 1){
      Serial.print("Gx :  ,Gy :  ,Gz :  ,");
    }
    if(MAG_RECORD == 1){
      Serial.print("Mx :  ,My :  ,Mz :  ,");
    }
    if(ORIENTATION_RECORD == 1){
      Serial.print("Pitch:,Roll: ,Yaw:  ,");
    }
    if(ALTITUDE_RECORD == 1){
      Serial.print("Barom:,");
    }
    if(TEMPERATURE_RECORD == 1){
      Serial.print("Temp: ,");
    }
    if(BATTERY_RECORD == 1){
      Serial.print("Volt: ,");
    }
    Serial.println();
    Serial.println("_______________________________________________________________________________________________________________________");

  }
//-------------------------------------------------------------------------//
//*****************************   Data Retrieve   *************************//
//-------------------------------------------------------------------------//
static void Data_Retrieve(){
    if(GPS_RECORD == 1 || GPS_SEND == 1){
      gps.f_get_position(&flat, &flon, &age);
    }
    if(ACCEL_RECORD == 1 || ACCEL_SEND == 1 || GYRO_RECORD == 1 || GYRO_SEND == 1 || MAG_RECORD == 1 || MAG_SEND == 1 || ORIENTATION_RECORD == 1 || ORIENTATION_SEND == 1 || ALTITUDE_RECORD == 1 || ALTITUDE_SEND == 1 || TEMPERATURE_RECORD == 1 || TEMPERATURE_SEND == 1){
      IMU.readSensor();

    }
    if(ACCEL_RECORD == 1 || ACCEL_SEND == 1 || GYRO_RECORD == 1 || GYRO_SEND == 1 || MAG_RECORD == 1 || MAG_SEND == 1){
      ax = IMU.getAccelX_mss()/9.8;
      ay = IMU.getAccelY_mss()/9.8;
      az = IMU.getAccelZ_mss()/9.8;

      gx = IMU.getGyroX_rads()*114.59;
      gy = IMU.getGyroY_rads()*114.59;
      gz = IMU.getGyroZ_rads()*114.59;

      mx = IMU.getMagX_uT();
      my = IMU.getMagY_uT();
      mz = IMU.getMagZ_uT();
    }
    if((ACCEL_RECORD == 1 || ACCEL_SEND == 1 || GYRO_RECORD == 1 || GYRO_SEND == 1 || MAG_RECORD == 1 || MAG_SEND == 1)&&(ORIENTATION_RECORD == 1 || ORIENTATION_SEND == 1)){
      filter.updateIMU(gx, gy, gz, ax, ay, az);
      roll = filter.getRoll();
      pitch = filter.getPitch();
      yaw = filter.getYaw();
    }
    if((ACCEL_RECORD != 1 && ACCEL_SEND != 1 && GYRO_RECORD != 1 && GYRO_SEND != 1 && MAG_RECORD != 1 && MAG_SEND && 1) && (ORIENTATION_RECORD == 1 || ORIENTATION_SEND == 1)){
      ax = IMU.getAccelX_mss()/9.8;
      ay = IMU.getAccelY_mss()/9.8;
      az = IMU.getAccelZ_mss()/9.8;

      gx = IMU.getGyroX_rads()*114.59;
      gy = IMU.getGyroY_rads()*114.59;
      gz = IMU.getGyroZ_rads()*114.59;

      mx = IMU.getMagX_uT();
      my = IMU.getMagY_uT();
      mz = IMU.getMagZ_uT();

      temp = IMU.getTemperature_C();
      filter.updateIMU(gx, gy, gz, ax, ay, az);
      roll = filter.getRoll();
      pitch = filter.getPitch();
      yaw = filter.getYaw();
      if(ALTITUDE_SEND == 1 || ALTITUDE_RECORD == 1){
          long realPressure = ms5611.readPressure();
      }
    }
    if(ALTITUDE_RECORD == 1 || ALTITUDE_SEND == 1){
      barom = ms5611.getAltitude(realPressure);
    }
    if(TEMPERATURE_RECORD == 1 || TEMPERATURE_SEND == 1){
      temp = IMU.getTemperature_C();
    }
    if(BATTERY_RECORD == 1 || BATTERY_SEND == 1){
      // read the input on analog pin 0:
      AnalogsensorValue = analogRead(ADC_Pin);
      // Convert the ADC reading (which goes from 0 - 1023) to a voltage reading (0 - 3.3V):
      voltage = 2 * AnalogsensorValue * (3.3 / 1023.0);
    }
  }

//-------------------------------------------------------------------------//
//*****************************   Data Retrieve   *************************//
//-------------------------------------------------------------------------//
void Button_Press() {
  static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    // If interrupts come faster than 200ms, assume it's a bounce and ignore
    if(interrupt_time - last_interrupt_time > 300){
      state = !state;
    }
  last_interrupt_time = interrupt_time;
  }
