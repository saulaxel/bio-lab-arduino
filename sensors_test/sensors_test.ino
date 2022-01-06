#include "sensors.h"

int* light_sensor;
int* sharp_sensor;

void setup() {

	Serial.begin(9600);
}

void loop() {
	
	read_sensors_data();
	
	light_sensor = get_light_sensors();
	Serial.print("[");
        Serial.print(light_sensor[0]);
	for(int i=1; i<8; i++)
	{
	  Serial.print(", ");
          Serial.print(light_sensor[i]);
	}
	Serial.print("]\t");//*/

	sharp_sensor = get_sharp_sensors();
	Serial.print("[");
        Serial.print(sharp_sensor[0]);
	for(int i=1; i<8; i++)
	{
	  Serial.print(", ");
          Serial.print(sharp_sensor[i]);
	}
	Serial.println("]");//*/
	
	delay(200);	
}
