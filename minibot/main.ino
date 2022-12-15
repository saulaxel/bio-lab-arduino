#include "ros.h"
#include "sensors.h"
#include "motors_speed.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#define BAUD 200000

ros::NodeHandle nh;

int timer = 0;
long encoder_data[2];
float left_speed, right_speed;

std_msgs::Int16 batt_perc_msg;
std_msgs::Int32MultiArray encoders_msg;
std_msgs::Int16MultiArray line_sensors_msg;
std_msgs::Int16MultiArray light_sensors_msg;
std_msgs::Int16MultiArray sharp_sensors_msg;
std_msgs::Float32MultiArray imu_sensors_msg;

void motorsSpeedCallback(const std_msgs::Float32MultiArray& msg);

ros::Publisher battPercPub("/battery_data", &batt_perc_msg);
ros::Publisher encodersPub("/encoders_data", &encoders_msg);
ros::Publisher lineSensorsPub("/line_sensors", &line_sensors_msg);
ros::Publisher lightSensorsPub("/light_sensors", &light_sensors_msg);
ros::Publisher sharpSensorsPub("/sharp_sensors", &sharp_sensors_msg);
ros::Publisher imuSensorPub("/imu_sensors", &imu_sensors_msg);
ros::Subscriber<std_msgs::Float32MultiArray> subMotorsSpeed("/speed_motors", motorsSpeedCallback);

void publish_battery() {
  batt_perc_msg.data = get_battery_sensor();
  battPercPub.publish(&batt_perc_msg);
}

void publish_encoders() {

  encoders_msg.data_length = 2;
  encoder_data[0] = encoder_left();
  encoder_data[1] = encoder_right();
  encoders_msg.data = encoder_data;

  encodersPub.publish(&encoders_msg);
}

  void publish_sensors_data() {

    read_sensors_data();
    line_sensors_msg.data_length  = 3;
    light_sensors_msg.data_length = 8;
    sharp_sensors_msg.data_length = 7;
    imu_sensors_msg.data_length = 9;

    line_sensors_msg.data  = get_line_sensors();
    light_sensors_msg.data = get_light_sensors();
    sharp_sensors_msg.data = get_sharp_sensors();
    imu_sensors_msg.data = get_imu_sensors();

    lineSensorsPub.publish(&line_sensors_msg);
    lightSensorsPub.publish(&light_sensors_msg);
    sharpSensorsPub.publish(&sharp_sensors_msg);
    imuSensorPub.publish(&imu_sensors_msg);
  }

void motorsSpeedCallback(const std_msgs::Float32MultiArray& msg){

  left_speed  = msg.data[0];
  right_speed = msg.data[1];
  timer = 0;
}

void setup() {
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();

  nh.advertise(battPercPub);
  nh.advertise(encodersPub);
  nh.advertise(lineSensorsPub);
  nh.advertise(lightSensorsPub);
  nh.advertise(sharpSensorsPub);
  nh.advertise(imuSensorPub);
  nh.subscribe(subMotorsSpeed);


  while (!nh.connected()) { nh.spinOnce(); }

  bool params_error = false;
  char log_msg[50];
  char Kp[8], Ki[8], Kd[8];
  float left_pid1[3], left_pid2[3], right_pid1[3], right_pid2[3];

  if(! nh.getParam("/left_pid1", left_pid1, 3))   { params_error = true; }
  if(! nh.getParam("/left_pid2", left_pid2, 3))   { params_error = true; }
  if(! nh.getParam("/right_pid1", right_pid1, 3)) { params_error = true; }
  if(! nh.getParam("/right_pid2", right_pid2, 3)) { params_error = true; }

  if(params_error) { nh.logerror("File: ~/Minibot/catkin_ws/src/rob2w_description/params/pid.yaml did not load values, check it please"); }

  dtostrf(left_pid1[0], 6, 2, Kp);
  dtostrf(left_pid1[1], 6, 5, Ki);
  dtostrf(left_pid1[2], 6, 4, Kd);
  sprintf(log_msg, "Left pid 1->[%s, %s, %s]", Kp, Ki, Kd);
  nh.loginfo(log_msg);

  dtostrf(left_pid2[0], 6, 2, Kp);
  dtostrf(left_pid2[1], 6, 5, Ki);
  dtostrf(left_pid2[2], 6, 4, Kd);
  sprintf(log_msg, "Left pid 2->[%s, %s, %s]", Kp, Ki, Kd);
  nh.loginfo(log_msg);

  dtostrf(right_pid1[0], 6, 2, Kp);
  dtostrf(right_pid1[1], 6, 5, Ki);
  dtostrf(right_pid1[2], 6, 4, Kd);
  sprintf(log_msg, "Right pid 1->[%s, %s, %s]", Kp, Ki, Kd);
  nh.loginfo(log_msg);

  dtostrf(right_pid2[0], 6, 2, Kp);
  dtostrf(right_pid2[1], 6, 5, Ki);
  dtostrf(right_pid2[2], 6, 4, Kd);
  sprintf(log_msg, "Right pid 2->[%s, %s, %s]", Kp, Ki, Kd);
  nh.loginfo(log_msg);

  set_motors(left_pid1, left_pid2, right_pid1, right_pid2);
}


void loop() {

  publish_battery();
  publish_encoders();
  publish_sensors_data();
  motors_speed(left_speed, right_speed);

  if(timer < 200) timer++;
  else motors_speed(0, 0);

  nh.spinOnce();
  delay(15);
}
