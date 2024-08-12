#include "line.h"
#define left_sensor   A0
#define center_sensor A2
#define right_sensor  A3
#define emit 11
#define number_of_pins 3
#define threshhold 1500
//unsigned long end_time_ls[number_of_pins]
int list_sensor[number_of_pins] = {left_sensor, center_sensor, right_sensor};
float ls_reading[number_of_pins];
float error;
Linesensor ls;



void setup() {
  
}

void loop(){
  ls.read_all_sensor()
  Serial.println(left_sensor);Serial.println(",");
  Serial.println(center_sensor);Serial.println(",");
  Serial.println(right_sensor);Serial.println(",");

}
