#include "Arduino.h"

#define R_LSENSOR 12
#define L_LSENSOR 13
#define C_LSENSOR 22

// Para la unidad de medición inercial (IMU)
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;

int ldr[8];
int sharp[8];
int line[3];
float data_imu[9];
int battery;

void set_line_sensors() {
  pinMode(R_LSENSOR, INPUT);
  pinMode(L_LSENSOR, INPUT);
  pinMode(C_LSENSOR, INPUT);
}

void read_sensors_data() {

  for(int i=0; i<8; i++)
    ldr[i] = analogRead(i);

  for(int i=8; i<15; i++)
    sharp[i-8] = analogRead(i);

  battery = analogRead(15);
  line[0] = digitalRead(R_LSENSOR);
  line[1] = digitalRead(L_LSENSOR);
  line[2] = digitalRead(C_LSENSOR);

  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
    data_imu[0] = imu.calcGyro(imu.gx);
    data_imu[1] = imu.calcGyro(imu.gy);
    data_imu[2] = imu.calcGyro(imu.gz);
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
    data_imu[3] = imu.calcAccel(imu.ax);
    data_imu[4] = imu.calcAccel(imu.ay);
    data_imu[5] = imu.calcAccel(imu.az);
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();

    data_imu[6] = imu.calcMag(imu.mx);
    data_imu[7] = imu.calcMag(imu.my);
    data_imu[8] = imu.calcMag(imu.mz);
  }
}

int get_battery_sensor() {
  return battery;
}

int* get_light_sensors() {
  return ldr;
}

int* get_sharp_sensors() {
  return sharp;
}

int* get_line_sensors()  {
  return line;
}

float* get_imu_sensors() {
  return data_imu;
}
