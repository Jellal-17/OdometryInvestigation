#ifndef _MT_H
#define _MT_H
#include "encod.h"
# define L_PWM 10
# define L_DIR 16
# define R_PWM 9
# define R_DIR 15
float l_p ;
float r_p ;
# define FWD LOW
# define REV HIGH
#define PWM_UPPER 60
#define PWM_LOWER 15


class Motor {

  public:

    Motor() {

    }
    void initialize() {
      pinMode(L_PWM, OUTPUT);
      pinMode(L_DIR, OUTPUT);
      pinMode(R_PWM, OUTPUT);
      pinMode(R_DIR, OUTPUT);
    }

    float setLeftSpeedLimit(float l_pwm) {
      if (l_pwm == 0) {
        l_pwm = 0;
      }
      else {
        if (l_pwm > PWM_UPPER) {
          l_pwm = PWM_UPPER;
        }
        else if (l_pwm < -PWM_UPPER) {
          l_pwm = -PWM_UPPER;
        }
        else if (l_pwm > 0 && l_pwm < PWM_LOWER) {
          l_pwm = PWM_LOWER;
        }
        else if (l_pwm > -PWM_LOWER && l_pwm < 0) {
          l_pwm = -PWM_LOWER;
        }
      }
      return l_pwm;
    }

    float setRightSpeedLimit(float r_pwm) {
      if (r_pwm == 0) {
        r_pwm = 0;
      }
      else {
        if (r_pwm > PWM_UPPER) {
          r_pwm = PWM_UPPER;
        }
        else if (r_pwm < -PWM_UPPER) {
          r_pwm = -PWM_UPPER;
        }

        else if (r_pwm > 0 && r_pwm < PWM_LOWER) {
          r_pwm = PWM_LOWER;
        }
        else if (r_pwm > -PWM_LOWER && r_pwm < 0) {
          r_pwm = -PWM_LOWER;
        }
      }
      return r_pwm;
    }

    void setMotor(float l_p, float r_p) {
      l_p = setLeftSpeedLimit(l_p);
      r_p = setRightSpeedLimit(r_p);
      if (l_p >= 0 && r_p >= 0) {
        analogWrite(L_PWM, abs(l_p));
        analogWrite(R_PWM, abs(r_p));

        digitalWrite(L_DIR, FWD);
        digitalWrite(R_DIR, FWD);
      }

      else if (l_p < 0 && r_p < 0) {
        analogWrite(L_PWM, abs(l_p));
        analogWrite(R_PWM, abs(r_p));

        digitalWrite(L_DIR, REV);
        digitalWrite(R_DIR, REV);
      }

      // right turn
      else if (l_p >= 0 && r_p < 0) {
        analogWrite(L_PWM, abs(l_p));
        analogWrite(R_PWM, abs(r_p));

        digitalWrite(L_DIR, FWD);
        digitalWrite(R_DIR, REV);

      }

      // left turn
      else if (l_p < 0 && r_p >= 0) {
        analogWrite(L_PWM, abs(l_p));
        analogWrite(R_PWM, abs(r_p));

        digitalWrite(L_DIR, REV);
        digitalWrite(R_DIR, FWD);
      }

      else {
        analogWrite( L_PWM, 0);
        analogWrite( R_PWM, 0);
      }
    }

    void halt() {
      analogWrite( L_PWM, 0);
      analogWrite( R_PWM, 0);
//      while (1) {
//        Serial.println("Program Halted");
//        delay(500);
//      }
    }



};

#endif
