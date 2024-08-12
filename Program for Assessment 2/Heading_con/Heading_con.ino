#include "MT.h"
#include "LS.h"
#include "encod.h"
#include "Kine.h"
#include "pid.h"
# include <USBCore.h>
#include <EEPROM.h>
u8 USB_SendSpace(u8 ep);
# define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)
#define BUTTON_A_PIN 14
#define BUTTON_B_PIN 30


LineSensor ls;
Motor motor;
Kinematics kt;
PID_c spd_pid_right;
PID_c spd_pid_left;
PID_c line_following;

int state;
bool line = true;
double  angle;

#define LINE_UPDATE 20
#define MOTOR_UPDATE 20
#define BUMP_UPDATE 20
#define ARRAY_UPDATE 200

# define RESULTS_DIM 24
long results[RESULTS_DIM];

float ave_el_spd = 0;
float ave_er_spd = 0;
unsigned long lineSensor_ts = millis();
unsigned long bumpSensor_ts = millis();
unsigned long motor_ts = millis();
unsigned long array_t = millis();
unsigned long updates_ts = millis();
int inta = 0;
int intb = 0;
long count_el_last;
long count_er_last;

void setup() {

  //  enableBUMPs();
  ls.initialize();
  motor.initialize();
  setupEncoder0();
  setupEncoder1();
  spd_pid_left.initialize( 25.0, 0.0 , 0.0);
  spd_pid_right.initialize( 25.0, 0.0 , 0.0);
  ls.led_init();
  long count_el_last = count_l;
  long count_er_last = count_r;
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  pinMode(BUTTON_B_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  int state = 0;
  if ( SERIAL_ACTIVE )Serial.println("***RESET***");
}


void loop() {

  if (state == 0) {
    delay(1000);
    kt.updates();
    results[0] = Xt;
    results[1] = Yt;
    updateState();
  }
  
  else if (state == 1) {
    motorSt(35);
//    motorSt(50);
//    motorSt(65);
    bumpSense();
  }

else if (state == 2) {
  motorRotate(180);
  kt.updates();
  results[4] = Xt;
  results[5] = Yt;
}
else if (state == 3) {
  kt.updates();

  if (Xt <= 3.1) {
    motor.setMotor(0, 0);
    kt.updates();
    results[6] = Xt;
    results[7] = Yt;
    updateState();
  }

  else {
    motorStraight(1.0, 1.04);
    kt.updates();
    Serial.println(Xt);
  }
}
else {
  motor.halt();
  reportResultsOverSerial();
  delay(1000);
}
}

void updateState() {
  state = state + 1 ;

}
void motorSt(float speeds) {
  float c_d = count_l - count_r;
  float pwml = spd_pid_left.update( 0, c_d);
  motor.setMotor(speeds - 0.46 * (count_l - count_r), speeds - 0.5 * (count_r - count_l));
}

void motorStraight(float dL, float dR) {
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

    diff_e1 = (count_l - count_el_last) * kt.d_p_c;
    count_el_last = count_l;

    el_speed = (float)diff_e1;
    el_speed /= (float)elapsed; // actual elapsed ms.

    // low pass filter
    ave_el_spd = (ave_el_spd * 0.4) + (el_speed * 0.6);

    //right
    long diff_er;
    float er_speed;
    diff_er = (count_r - count_er_last) * kt.d_p_c;
    count_er_last = count_r;

    er_speed = (float)diff_er;
    er_speed /= (float)elapsed; // actual elapsed ms.

    // low pass filter
    ave_er_spd = (ave_er_spd * 0.4) + (er_speed * 0.6);

    float pwml;
    float pwmr;


    float demandL = map(dL, 15, 30, 1.0, 2.0);
    float demandR = map(dR, 15, 30, 1.0, 2.0);
    pwml = spd_pid_left.update( demandL, ave_el_spd );
    pwmr = spd_pid_right.update( demandR, ave_er_spd);
    motor.setMotor(pwml, pwmr);
  }
}

void motorRotate(float a) {
  kt.updates();
  if ( Tt <= a) {
    kt.updates();
    motor.setMotor(20, -20);
  }
  else {
    motor.setMotor(0, 0);
    updateState();
  }
}


void lineFollowing() {

  unsigned long elapsed_t;
  unsigned long current_ts;
  current_ts = millis();
  float e_line;

  elapsed_t = current_ts - lineSensor_ts;
  if (elapsed_t > LINE_UPDATE) {
    ls.parSR();
    e_line = ls.getLE();
    lineSensor_ts = millis();
  }
  //  float turn_power = ;
  float turn_power = ls.turn_pwm();
  //    float turn_power = kt.C_error();

  elapsed_t = current_ts - motor_ts;
  if (elapsed_t > MOTOR_UPDATE) {
    motor.setMotor((18 - turn_power), (18 + turn_power));
    motor_ts = millis();
  }

}

void bumpSense() {
  unsigned long elapsed_t;
  unsigned long current_ts;
  current_ts = millis();

  elapsed_t = current_ts - bumpSensor_ts;
  if (elapsed_t > BUMP_UPDATE) {
    ls.bumpreading();
    bumpSensor_ts = millis();

    if (bp_read[0] >= 1125 || bp_read[1] >= 1125) {
      motor.setMotor(0, 0);
      kt.updates();
      results[1] = Xt;
      results[2] = Yt;
      updateState();
    }
  }
}

void reportResultsOverSerial() {
  if ( SERIAL_ACTIVE ) Serial.print( "Time(ms): " );
  if ( SERIAL_ACTIVE ) Serial.println( millis() );
  delay(1);

  int i, j;
  for ( i = 0; i < 9; i++ ) {
    if ( SERIAL_ACTIVE ) Serial.print( results[i]);
    delay(1);
    if ( SERIAL_ACTIVE ) Serial.print( "," );
    delay(1);
  }
  if ( SERIAL_ACTIVE ) Serial.print( "\n" ); 

  if ( SERIAL_ACTIVE ) Serial.println( "---End of Results ---\n\n" );

}

void writeGridToEEPROM() {
  int x, address;

  address = 0;
  for ( x = 0; x < 25; x++ ) {
    EEPROM.update( address, results[x] );
  }

}

void readFromEEPROM() {
  int x, address;
  address = 0;
  for ( x = 0; x < 25; x++ ) {
    long value = EEPROM.read( address );
    Serial.print( value );
    Serial.print(",");
    address++;
  }
  Serial.print("\n");
  Serial.println("***********\n\n");
}
