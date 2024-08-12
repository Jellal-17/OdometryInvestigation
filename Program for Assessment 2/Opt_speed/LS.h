#ifndef _LS_H
#define _LS_H

#define LB 4
#define RB 5
#define LS_L A0
#define LS_C A2
#define LS_R A3
#define EM 11
#define RED 17
#define YEL 13
#define GRN 30
#define BUZ 6


int NB_LS_PINS = 3;
int ls_pins[5] = {LS_L, LS_C, LS_R};
int ls_read[5];

int NB_BP_PINS = 2;
int bp_pins[] = {LB, RB};
int bp_read[3];

float bias = 0;
float avg;
float L_L;
float L_C;
float L_R;
float w_r;
float w_l;
bool l1;
class LineSensor {
  public:


    LineSensor() {

    }
    void led_init() {
      pinMode(RED, OUTPUT);
      pinMode(YEL, OUTPUT);
      pinMode(GRN, OUTPUT);
      pinMode(BUZ, OUTPUT);
    }

    void initialize() {

      //    pinMode(EM, OUTPUT);
      pinMode(LB, INPUT);
      pinMode(RB, INPUT);
      pinMode(LS_L, INPUT);
      pinMode(LS_C, INPUT);
      pinMode(LS_R, INPUT);
      //   digitalWrite(EM, HIGH);

      Serial.begin(9600);
      Serial.print("***START***");
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



    float getLE() {
      float sum1 = 0;
      float c = 0;
      float sum = 0;
      float sum2 = 0;
      int count = 0;

      float e_line;

      parSR();


      for (int i = 0; i < NB_LS_PINS; i++) {
        sum = sum + ls_read[i];
      }
      if (ls_read[1] < 1500) {
        L_L = ls_read[0] / sum;
        L_C = ls_read[1] / sum;
//        L_R = (ls_read[2] - 30) / sum;
                L_R = (ls_read[2]) / sum;
      }

      if (ls_read[1] >= 1500) {
        L_L = ls_read[0] / sum;
        L_C = (ls_read[1]) / sum;

//        L_R = (ls_read[2] - 50) / sum;
                L_R = (ls_read[2]) / sum;
      }

      w_r = L_R + (L_C * 0.5);
      w_l = L_L + (L_C * 0.5);
      e_line = w_l - w_r;

      return e_line;
    }

    float turn_pwm() {

      float  Kt = 100;
      float e_line = getLE();
      float tp = Kt * e_line;

      return tp;
    }

    bool lf() {
      if (ls_read[1] >= 1000) {
        l1 = true;
      }
      else if (ls_read[1] < 1000 && ls_read[0] < 1000 && ls_read[2] < 1000) {
        l1 = false;
      }
      return l1;
    }

};


#endif
