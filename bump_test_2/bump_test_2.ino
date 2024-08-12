//#include "MT.h"
#define LM 12
#define LB 4
#define RB 5
#define EM 11

#define LS_L A0
#define LS_C A2
#define LS_R A3

//Motor mt;

int NB_LS_PINS = 3;
int ls_pins[5] = {LS_L, LS_C, LS_R};
int ls_read[5];

int NB_BP_PINS = 2;
int bp_pins[] = {LB, RB};
int bp_read[3];

void setup() {

  //  enableBUMPs();
  pinMode(LB, INPUT);
  pinMode(RB, INPUT);
  pinMode(LS_L, INPUT);
  pinMode(LS_C, INPUT);
  pinMode(LS_R, INPUT);

  Serial.begin(9600);
  Serial.print("RESET");
}


void loop() {
  bumpreading();
  // mt.setMotor(20, -20);
  Serial.print(bp_read[0]);
  Serial.print(",");
  Serial.println(bp_read[1]);
  delay(200);

  parSR();
  Serial.print(ls_read[0]);
  Serial.print(",");
  Serial.print(ls_read[1]);
  Serial.print(",");
  Serial.println(ls_read[2]);
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

void chargeCapacitors1() {
  for (int i = 0; i < NB_LS_PINS ; i++) {
    pinMode(ls_pins[i], OUTPUT);
    digitalWrite(ls_pins[i], HIGH);
  }

  delayMicroseconds(10);

  for (int i = 0; i < NB_LS_PINS ; i++) {
    pinMode(ls_pins[i], INPUT);
  }
}

void enableIRLEDs() {
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
  enableBUMPs();
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

void parSR( ) {
  int which;
  int count;
  enableIRLEDs();
  chargeCapacitors1();


  unsigned long start_time;
  start_time = micros();

  unsigned long end_time_ls[NB_LS_PINS];

  for (which = 0; which < NB_LS_PINS ; which++) {
    end_time_ls[which] = 0 ;
  }

  bool done = false;
  count = NB_LS_PINS;

  while (done != true) {
    for (which = 0; which < NB_LS_PINS ; which++) {

      if ( end_time_ls[which] == 0) {
        if (digitalRead(ls_pins[which]) == LOW) {
          end_time_ls[which] = micros();
          count = count - 1;
        }
      }

      if ( count == 0) {
        done = true;
      }// if all sensors read.
    }//for.
  }//while.

  for (which = 0; which < NB_LS_PINS ; which++) {
    unsigned long elapsed_time;
    elapsed_time = end_time_ls[which] - start_time;
    ls_read[which] = (float)elapsed_time;
  }
}// end of psr
