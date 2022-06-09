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

void motorsSpeedCallback(const std_msgs::Float32MultiArray& msg);

ros::Publisher battPercPub("/battery_data", &batt_perc_msg);
ros::Publisher encodersPub("/encoders_data", &encoders_msg);
ros::Publisher lineSensorsPub("/line_sensors", &line_sensors_msg);
ros::Publisher lightSensorsPub("/light_sensors", &light_sensors_msg);
ros::Publisher sharpSensorsPub("/sharp_sensors", &sharp_sensors_msg);
ros::Subscriber<std_msgs::Float32MultiArray> subMotorsSpeed("/speed_motors", motorsSpeedCallback);

void publish_battery() {
  batt_perc_msg.data = get_battery_sensor();
  battPercPub.publish(&batt_perc_msg);
}

void publish_encoders(){
  
  encoders_msg.data_length = 2;
  encoder_data[0] = encoder_left();
  encoder_data[1] = encoder_right();
  encoders_msg.data = encoder_data;
  
  encodersPub.publish(&encoders_msg);
}

void publish_sensors_data(){

  read_sensors_data();
  line_sensors_msg.data_length  = 2;
  light_sensors_msg.data_length = 8;
  sharp_sensors_msg.data_length = 7;
  
  line_sensors_msg.data  = get_line_sensors();
  light_sensors_msg.data = get_light_sensors();
  sharp_sensors_msg.data = get_sharp_sensors();

  lineSensorsPub.publish(&line_sensors_msg);
  lightSensorsPub.publish(&light_sensors_msg);
  sharpSensorsPub.publish(&sharp_sensors_msg);  
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
  nh.subscribe(subMotorsSpeed);

  set_motors(); 
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
