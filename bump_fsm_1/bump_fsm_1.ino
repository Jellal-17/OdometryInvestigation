#include "MT.h"
#include "LS.h"
#include "encod.h"
#include "Kine.h"
# include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
# define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

LineSensor ls;
Motor motor;
Kinematics kt;
int state;
bool line = true;
double  angle;

#define LINE_UPDATE 20
#define MOTOR_UPDATE 20
#define BUMP_UPDATE 20
#define ARRAY_UPDATE 200

# define RESULTS_DIM 10
long results[RESULTS_DIM];

unsigned long lineSensor_ts = millis();
unsigned long bumpSensor_ts = millis();
unsigned long motor_ts = millis();
unsigned long array_t = millis();
int inta = 0;
int intb = 0;

void setup() {

  //  enableBUMPs();
  ls.initialize();
  motor.initialize();
  setupEncoder0();
  setupEncoder1();
  ls.led_init();
  int state = 0;
  Serial.begin(9600);
  if ( SERIAL_ACTIVE )Serial.println("***RESET***");
}


void loop() {

  Serial.println(state);

  if (state == 0) {
    delay(2000);
    kt.updates();
    results[0] = Xt;
    updateState();
  }
  else if (state == 1) {
    lineFollowing();
    bumpSense();
  }
  else if (state == 2) {
    motor.setMotor(30, -30);
    delay(1000);
    motor.setMotor(0, 0);
    kt.updates();
    results[2] = Xt;
    updateState();
  }
  else if (state == 3) {

    lineFollowing();
    bool linetrue = ls.lf();
    if (linetrue == false) {
      //      delay(200);
      motor.setMotor(0, 0);
      kt.updates();
      results[3] = Xt;
      updateState();
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

void motorRotate(float a) {
  kt.updates();

  //clockwise
  if (angle > 0) {
    if ( Tr <= a) {
      kt.updates();
      motor.setMotor(20, -20);
    }
    else {
      motor.setMotor(0, 0);
      updateState();
    }
  }

  // anti-clockwise
  else if (angle < 0) {
    if ( Tr >= a) {
      kt.updates();
      motor.setMotor(-20, 20);

    }
    else {
      motor.setMotor(0, 0);
      updateState();
    }

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

    if (bp_read[0] >= 740 || bp_read[1] >= 740) {
      motor.setMotor(0, 0);
      kt.updates();
      results[1] = Xt;
      updateState();
    }

    else {
      Serial.print(bp_read[0]);
      Serial.print(",");
      Serial.println(bp_read[1]);
      motor.setMotor(18, 18);
      digitalWrite(RED, HIGH);
      digitalWrite(YEL, HIGH);
      delay(200);
      digitalWrite(RED, LOW);
      digitalWrite(YEL, LOW);
      delay(200);
    }
  }


}

void reportResultsOverSerial() {
  if ( SERIAL_ACTIVE ) Serial.print( "Time(ms): " );
  if ( SERIAL_ACTIVE ) Serial.println( millis() );
  delay(1);


  // Loop through array to print all
  // results collected
  int i, j;
  for ( i = 0; i < 10; i++ ) { // col

    // Comma seperated values, to 2 decimal places
    if ( SERIAL_ACTIVE ) Serial.print( results[i]);
    delay(1);
    if ( SERIAL_ACTIVE ) Serial.print( "," );
    delay(1);
  }
  if ( SERIAL_ACTIVE ) Serial.print( "\n" ); // new row


  if ( SERIAL_ACTIVE ) Serial.println( "---End of Results ---\n\n" );

}
