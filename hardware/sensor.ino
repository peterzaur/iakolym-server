#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_LSM9DS0.h>

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11
#define mySerial Serial1 // RX, TX

Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup()
{
   /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    mySerial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  mySerial.println(F("Found LSM9DS0 9DOF"));
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  mySerial.begin(9600);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop()
{

  while (!mySerial.available());
  
  sensors_event_t accel, mag, gyro, temp;
  sensors_vec_t orientation;
  
  lsm.getEvent(&accel, &mag, &gyro, &temp); 
  

  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag, &orientation))
  {
    mySerial.println(orientation.heading);
  }
  
  delay(1000);
}
