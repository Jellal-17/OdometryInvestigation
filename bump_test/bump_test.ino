#include "MT.h"
#define LM 12
#define LB 4
#define RB 5
#define EM 11

Motor mt;

int NB_BP_PINS = 2;
int bp_pins[] = {LB, RB};
int bp_read[3];

void setup() {

  enableBUMPs();
  pinMode(LB, INPUT);
  pinMode(RB, INPUT);

  Serial.begin(9600);
  Serial.print("RESET");
}


void loop() {
  bumpreading();
  //    mt.setMotor(20, -20);
  Serial.print(bp_read[0]);
  Serial.print(",");
  Serial.println(bp_read[1]);
  delay(200);
}

void chargeCapacitors() {
  for (int i = 0; i < NB_BP_PINS ; i++) {
    pinMode(bp_pins[i], OUTPUT);
    digitalWrite(bp_pins[i], HIGH);
  }

  delayMicroseconds(10);

  for (int i = 0; i < NB_BP_PINS ; i++) {
    pinMode(bp_pins[i], INPUT);
  }
}
void disableBUMPs() {
  pinMode(EM, OUTPUT);
  digitalWrite(EM, HIGH);

}
void enableBUMPs() {
  pinMode(EM, OUTPUT);
  digitalWrite(EM, LOW);
}


void bumpreading() {
  int which;
  int count;

  chargeCapacitors();


  unsigned long start_time;
  start_time = micros();

  unsigned long end_time_ls[NB_BP_PINS];

  for (which = 0; which < NB_BP_PINS ; which++) {
    end_time_ls[which] = 0 ;
  }

  bool done = false;
  count = NB_BP_PINS;

  while (done != true) {
    for (which = 0; which < NB_BP_PINS ; which++) {

      if ( end_time_ls[which] == 0) {
        if (digitalRead(bp_pins[which]) == LOW) {
          end_time_ls[which] = micros();
          count = count - 1;
        }
      }

      if ( count == 0) {
        done = true;
      }// if all sensors read.
    }//for.
  }//while.

  for (which = 0; which < NB_BP_PINS ; which++) {
    unsigned long elapsed_time;
    elapsed_time = end_time_ls[which] - start_time;
    bp_read[which] = (float)elapsed_time;
  }
}
