#include "MT.h"
#include "encod.h"
#include "Kine.h"
#include "pid.h"

Motor motor;
Kinematics kt;
PID_c spd_pid_right;
PID_c spd_pid_left;
PID_c line_following;

float ave_el_spd = 0;
float ave_er_spd = 0;
long count_el_last;
long count_er_last;

unsigned long updates_ts = millis();

void setup() {
  motor.initialize();
  setupEncoder0();
  setupEncoder1();
  spd_pid_left.initialize( 50.0, 0.0 , 0.0);
  spd_pid_right.initialize( 50.0, 0.1 , 0.0);
    delay(1000);
    motor.setMotor( 20, 20);
}

void loop() {

  motorStraight(1.0,1.15);
//  float c_d = count_l - count_r;
//  float pwml = spd_pid_left.update( 0, c_d);
//  motor.setMotor(20-0.46*(count_l - count_r), 20-0.5*(count_r - count_l));


  //  motor.setMotor(30, 30);
  //  delay(200);
  //    motor.setMotor((20 + C), (20 - C));
  //    motorStraight(0, 0.1);
  //  float pwml;
  //  float pwmr;
  //  pwml = spd_pid_left.update( 0, C);
  //    pwmr = spd_pid_right.update(ave_er_spd, demandR);

  //  motor.setMotor((-pwml), (-pwml));
}

void motorStraight(float demandL, float demandR) {
  //  demandL = 0.8;
  //  demandR = 0.8;
  unsigned long elapsed;

  elapsed = millis() - updates_ts;

  // cal speed estimate.

  if (elapsed > 20) {
    updates_ts = millis();

    //left
    long diff_e1;
    float el_speed;
    diff_e1 = -count_l + count_el_last;
    count_el_last = count_l;

    el_speed = (float)diff_e1;
    el_speed /= (float)elapsed; // actual elapsed ms.

    // low pass filter
    ave_el_spd = (ave_el_spd * 0.7) + (el_speed * 0.3);

    //right
    long diff_er;
    float er_speed;
    diff_er = -count_r + count_er_last;
    count_er_last = count_r;

    er_speed = (float)diff_er;
    er_speed /= (float)elapsed; // actual elapsed ms.

    // low pass filter
    ave_er_spd = (ave_er_spd * 0.7) + (er_speed * 0.3);

    float pwml;
    float pwmr;
    pwml = spd_pid_left.update( ave_el_spd, demandL);
    pwmr = spd_pid_right.update(ave_er_spd, demandR);

    motor.setMotor(pwml, pwmr);
  }
}
