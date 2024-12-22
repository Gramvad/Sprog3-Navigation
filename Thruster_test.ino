
// Main Branch of TideBreak, automated surfaces vechichle. 

//Library setup
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_SensorLab.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_GPS.h>
#include <Adafruit_AHRS.h>

// AHRS Filter. pick your filter! slower == better quality output
//Adafruit_NXPSensorFusion filter; // slowest
//Adafruit_Madgwick filter;  // faster than NXP
Adafruit_Mahony filter;  // fastest/smalleset


#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

// AHRS Filter Variables
#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10
//#define AHRS_DEBUG_OUTPUT

uint32_t timestamp;

// GPS setup, using hardware serial
#define GPSSerial Serial1
Adafruit_GPS GPS(&Serial1);
char c;

// IMU setup
Adafruit_ICM20948 icm;
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// Constants Setup
#define SENSORS_RADS_TO_DPS M_PI / 180.0
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing

//Pin setup for thrustes
static const int thruster1 = 18, thruster2 = 19, thruster3 = 20, thruster4 = 21;
double CurrentLat, CurrentLon, TargetLat, TargetLon, OldLat, OldLon;

// Hard-iron calibration settings
const float hard_iron[3] = {
  -0.81,  -21.29,  42.90
};

// Soft-iron calibration settings
const float soft_iron[3][3] = {
  {  0.995,  0.004, 0.032  },
  {  0.004,  1.069, -0.013  },
  { -0.032, -0.0013,  0.942  }
};

// Magnetic declination from magnetic-declination.com
// East is positive ( ), west is negative (-)
// mag_decl = ( /-)(deg   min/60   sec/3600)
// Set to 0 to get magnetic heading instead of geo heading
const float mag_decl = 4.266;


//Debugging tools
bool MagCalibrate = false;
bool debug = false;


void setup(void) {

  // Set up Serial port and GPS
  Serial.begin(9600);
  GPS.begin(9600);

  // Select GPS settings
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);


  delay(1000);

  while (!Serial)
  delay(10); // will pause Zero, Leonardo, etc until serial console opens

  //IMU Initilize
  ICMSetup();

  //Set target GPS waypoint
  TargetLat = 54.9132;
  TargetLon = 9.7793;

  // Initilize AHRS Filter
  filter.begin(FILTER_UPDATE_RATE_HZ);
  
}

void loop() {
  

  clearGPS();

  //Check if the GPS has a lock, and recieved a message. If there is a new message store in a variable for reading
  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
  }

  GPS.parse(GPS.lastNMEA());

  //  Create a new normalized sensor event for IMU
  sensors_event_t accel_event;  
  sensors_event_t gyro_event;
  sensors_event_t mag_event;
  sensors_event_t temp_event;

  // Setup AHRS variables
  float roll, pitch, yaw;
  float gx, gy, gz;
  static uint8_t counter = 0;

  //Reset Thrusters
  analogWrite(thruster1, 0);
  analogWrite(thruster2, 0);
  analogWrite(thruster3, 0);
  analogWrite(thruster4, 0);

  //Read IMU values and store them in sensor events variables.
  icm.getEvent(&accel_event, &gyro_event, &temp_event, &mag_event);

  // Callibrating the IMU for the AHRS, based on a calibration file.
  //cal.calibrate(mag);
  //cal.calibrate(accel);
  //cal.calibrate(gyro);


  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gyro_event.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro_event.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro_event.gyro.z * SENSORS_RADS_TO_DPS;

  //Run the IMU data though the AHRS algorithm, storing the values in the pitch roll and yaw variables.
  filter.update(gx, gy, gz, 
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z, 
                mag_event.magnetic.x, mag_event.magnetic.y, mag_event.magnetic.z);


  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();


  // Set up variables for calculating compass heading
  static float hi_cal[3]; // Array for adding hard iron calibration to magnetometer values.
  static float heading = 0;
  
  // Put raw magnetometer readings into an array
  float mag_data[] = {
    mag_event.magnetic.x,
    mag_event.magnetic.y,
    mag_event.magnetic.z
  };

  // Apply hard-iron offsets
  for (uint8_t i = 0; i < 3; i++  ) {
    hi_cal[i] = mag_data[i] - hard_iron[i];
  };

  // Apply soft-iron scaling
  for (uint8_t i = 0; i < 3; i++  ) {
    mag_data[i] = (soft_iron[i][0] * hi_cal[0]) + 
                  (soft_iron[i][1] * hi_cal[1]) +
                  (soft_iron[i][2] * hi_cal[2]);
  };

  // Calculate angle for heading, assuming board is parallel to
  // the ground and  Y points toward heading.
  heading = -1 * (atan2(mag_data[0], mag_data[1]) * 180) / M_PI;

  // Apply magnetic declination to convert magnetic heading
  // to geographic heading
    heading  += mag_decl;

  // Convert heading to 0..360 degrees
  if (heading < 0) {
    heading  += 360;
  }

  // IF GPS has a fix, update the old GPS location, and store new ones. 
  if (GPS.fix) {
    OldLat = CurrentLat;
    OldLon = CurrentLon;
    CurrentLat = GPS.latitudeDegrees;
    CurrentLon = GPS.longitudeDegrees;
  }

  //Calculating the distance and bearing between newest GPS location and target, aswell as the newest GPS location and the last one.
  double targetDistance = calculateDistance(TargetLat, TargetLon, CurrentLat, CurrentLon);
  double targetBearing = calculateBearing(TargetLat, TargetLon, CurrentLat, CurrentLon);
  double oldDistance = calculateDistance(CurrentLat, CurrentLon, OldLat, OldLon);
  double oldBearing = calculateBearing(CurrentLat, CurrentLon, OldLat, OldLon);

  //Weighted average, since we trust the compass more than the AHRS calibration
  float weigthedHeading = (heading*1.2+yaw/2);

  //Thruster selection, and turning on the correct thrustes based on heading at 50% duty cycle.
  if (targetBearing < (weigthedHeading + 10) ) {
    Serial.println("Turning left");
    analogWrite(thruster1, 191);
    analogWrite(thruster3, 191);
  } else if ((weigthedHeading - 10) < targetBearing) {
    Serial.println("Turning right");
    analogWrite(thruster2, 191);
    analogWrite(thruster4, 191);
  } else {
    Serial.println("thrusting forward");
    analogWrite(thruster3, 191);
    analogWrite(thruster4, 191);
  }

  delay(2000);


  //debugging for GPS
  if (GPS.fix & debug) {
    Serial.print("Time: ");
    Serial.print(GPS.hour, DEC);
    Serial.print(':');
    Serial.print(GPS.minute, DEC);
    Serial.print(':');
    Serial.print(GPS.seconds, DEC);
    Serial.print('.');
    Serial.println(GPS.milliseconds);

    Serial.print("Date: ");
    Serial.print(GPS.day, DEC);
    Serial.print('/');
    Serial.print(GPS.month, DEC);
    Serial.print("/20");
    Serial.println(GPS.year, DEC);

    Serial.print("Fix: ");
    Serial.print(GPS.fix);
    Serial.print(" quality: ");
    Serial.println(GPS.fixquality);
    Serial.print("Satellites: ");
    Serial.println(GPS.satellites);
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4);
    Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4);
    Serial.println(GPS.lon);
    Serial.print("Google Maps location: ");
    Serial.print(GPS.latitudeDegrees, 4);
    Serial.print(", ");
    Serial.println(GPS.longitudeDegrees, 4);
  };

  // debugging for magnetometer calibration
  if (MagCalibrate) {
    Serial.print("Raw:");
    Serial.print(int(accel_event.acceleration.x*8192/9.8)); Serial.print(",");
    Serial.print(int(accel_event.acceleration.y*8192/9.8)); Serial.print(",");
    Serial.print(int(accel_event.acceleration.z*8192/9.8)); Serial.print(",");
    Serial.print(int(gyro_event.gyro.x*Adafruit_SensorLab::DEGREES_PER_RADIAN*16)); Serial.print(",");
    Serial.print(int(gyro_event.gyro.y*Adafruit_SensorLab::DEGREES_PER_RADIAN*16)); Serial.print(",");
    Serial.print(int(gyro_event.gyro.z*Adafruit_SensorLab::DEGREES_PER_RADIAN*16)); Serial.print(",");
    Serial.print(int(mag_data[0]*10)); Serial.print(",");
    Serial.print(int(mag_data[1]*10)); Serial.print(",");
    Serial.print(int(mag_data[2]*10)); Serial.println("");

    // unified data
    Serial.print("Uni:");
    Serial.print(accel_event.acceleration.x); Serial.print(",");
    Serial.print(accel_event.acceleration.y); Serial.print(",");
    Serial.print(accel_event.acceleration.z); Serial.print(",");
    Serial.print(gyro_event.gyro.x, 4); Serial.print(",");
    Serial.print(gyro_event.gyro.y, 4); Serial.print(",");
    Serial.print(gyro_event.gyro.z, 4); Serial.print(",");
    Serial.print(mag_data[0]); Serial.print(",");
    Serial.print(mag_data[1]); Serial.print(",");
    Serial.print(mag_data[2]); Serial.println("");

  };
}

double calculateDistance(double TargetLat, double TargetLon, double CurrentLat, double CurrentLon) {
  // distance between latitudes
  // and longitudes
  double dLat = (CurrentLat - TargetLat) * SENSORS_RADS_TO_DPS;
  double dLon = (CurrentLon - TargetLon) * SENSORS_RADS_TO_DPS;

  // convert to radians
  TargetLat = (TargetLat) * SENSORS_RADS_TO_DPS;
  CurrentLat = (CurrentLat) * SENSORS_RADS_TO_DPS;
 
  // apply formuler of haversine for distance calculation
  double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(TargetLat) * cos(CurrentLat);
  double rad = 6371;
  double c = 2 * asin(sqrt(a));
  double distance = rad * c;

  Serial.println(distance);
  return(distance);
}

double calculateBearing(double TargetLat, double TargetLon, double CurrentLat, double CurrentLon){
  
  //Delta lat and lon
  double dLat = (CurrentLat - TargetLat) * SENSORS_RADS_TO_DPS;
  double dLon = (CurrentLon - TargetLon) * SENSORS_RADS_TO_DPS;
  double x = cos(TargetLat)*sin(dLon);
  double y = cos(TargetLat)*sin(CurrentLat)-sin(TargetLat)*cos(CurrentLat)*cos(dLon);
  double beta = atan2(x,y);
  double bearing = degrees(beta);
  bearing = ( ((int)bearing + 360) % 360 );
  return bearing;
}

void clearGPS() {
  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());

  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
}

void ICMSetup() {
  Serial.println("Adafruit ICM20948 test!");
  
  // Try to initialize!
  if (!icm.begin_I2C()) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {
    Serial.println("Failed to find ICM20948 chip");
    
    while (1) {
    
    delay(10);
    }
  }

  Serial.println("ICM20948 Found!");
  // icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  
  switch (icm.getAccelRange()) {
    case ICM20948_ACCEL_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case ICM20948_ACCEL_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case ICM20948_ACCEL_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case ICM20948_ACCEL_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }

  Serial.println("OK");
  // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  Serial.print("Gyro range set to: ");
  
  switch (icm.getGyroRange()) {
    case ICM20948_GYRO_RANGE_250_DPS:
      Serial.println("250 degrees/s");
      break;
    case ICM20948_GYRO_RANGE_500_DPS:
      Serial.println("500 degrees/s");
      break;
    case ICM20948_GYRO_RANGE_1000_DPS:
      Serial.println("1000 degrees/s");
      break;
    case ICM20948_GYRO_RANGE_2000_DPS:
      Serial.println("2000 degrees/s");
      break;
  }

  // icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);
  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);
  
  // icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);
  
  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);
  
  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  Serial.print("Magnetometer data rate set to: ");
  
  switch (icm.getMagDataRate()) {
    case AK09916_MAG_DATARATE_SHUTDOWN:
      Serial.println("Shutdown");
      break;
    case AK09916_MAG_DATARATE_SINGLE:
      Serial.println("Single/One shot");
      break;
    case AK09916_MAG_DATARATE_10_HZ:
      Serial.println("10 Hz");
      break;
    case AK09916_MAG_DATARATE_20_HZ:
      Serial.println("20 Hz");
      break;
    case AK09916_MAG_DATARATE_50_HZ:
      Serial.println("50 Hz");
      break;
    case AK09916_MAG_DATARATE_100_HZ:
      Serial.println("100 Hz");
      break;
  }
  Serial.println();
}