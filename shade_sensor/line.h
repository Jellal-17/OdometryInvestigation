#ifndef _LINE_SENSOR_H
#define _LINE_SENSOR_H
#define left_sensor   A0
#define center_sensor A2
#define right_sensor  A3
#define emit 11


class Linesensor {
  public:

    Linesensor() {
    }
    void enableled() {
      pinMode(emit, OUTPUT);
      digitalWrite(emit, HIGH);
    }
    void disableled() {

      pinMode(emit, INPUT);

    }

    float readSingleSensor(int which) {
      enableled();
      pinMode(list_sensor[which], OUTPUT);
      digitalWrite(list_sensor[which], HIGH);
      delayMicroseconds(10);
      pinMode(list_sensor[which], INPUT);
      int timeout = 5000;
      float sensorReading;
      unsigned long start_time;
      start_time = micros();
      bool done = false;
      while ( done != true ) {
        unsigned long current_time = micros();
        unsigned long elapsed_time = current_time - start_time;
        if ( elapsed_time >= timeout ) {
          done = true;
          sensorReading = timeout;
        }
        if ( digitalRead(list_sensor[which]) == LOW ) {
          sensorReading = (float)elapsed_time;
          done = true;
        }
      }
      disableled();
      return sensorReading;
    }
    
    void read_all_sensor() {
      enableled();
      int which; // which sensor is left
      int count; // count how many sensors are finished reading
      // Charge the capacitor for all the sensors
      for (which = 0; which < number_of_pins; which++) {
        pinMode(list_sensor[which], OUTPUT);
        digitalWrite(list_sensor[which], HIGH);
      }
      delayMicroseconds(10);
      for (which = 0; which < number_of_pins; which++) {
        pinMode(list_sensor[which], INPUT);
      }
      // Record the charge start time
      unsigned long start_time;
      start_time = micros();
      // Which sensor has read it becomes non zero, starting with zero
      unsigned long end_time_ls[number_of_pins];
      for (which = 0; which < number_of_pins; which++) {
        end_time_ls[which] = 0;
      }
      bool done = false;
      count = number_of_pins;
      // Loop till sensors are finished reading
      while (done == false) {
        for (which = 0; which < number_of_pins; which++) {
          // Checking if the sensor reading is done, if 0 reading not done
          if (end_time_ls[which] == 0) {
            // Waiting for PIN to get LOW and read the discharge time and reduce the sensor count
            if (digitalRead(list_sensor[which]) == LOW) {
              end_time_ls[which] = micros();
              count = count - 1;
            }
          }
          if (count == 0) {
            done = true;
          }
        }
      }
      // Calculate the elapsed time for all the sensors and store
      for (which = 0; which < number_of_pins; which++) {
        unsigned long elapsed_time;
        elapsed_time = end_time_ls[which] - start_time;
        ls_reading[which] = (float)elapsed_time;

      }
      disableled();
    }
    
    bool line_detected() {
      read_all_sensor();
      if (ls_reading[0] > threshhold || ls_reading[1] > threshhold || ls_reading[2] > threshhold)
        return true;
      return false;
    }

    float eline () {
      read_all_sensor();
      float normalise = ls_reading[0] + ls_reading[1] + ls_reading[2];
      float left_normalise = ls_reading[0] / normalise;
      float center_normalise = ls_reading[1] / normalise;
      float right_normalise = ls_reading[2] / normalise;

      float weighted_left = left_normalise + (center_normalise * 0.5);
      float weighted_right = right_normalise + (center_normalise * 0.5);
      error = weighted_left - weighted_right;
      return error;
    }
};


#endif
