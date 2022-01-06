#include "Arduino.h"

int ldr[8];
int sharp[8];

void read_sensors_data() {

    for(int i=0; i<8; i++)
        ldr[i] = analogRead(i);

    for(int i=8; i<16; i++)
	sharp[i-8] = analogRead(i);
}

int* get_light_sensors(){
    return ldr;
}

int* get_sharp_sensors(){
    return sharp;
}
