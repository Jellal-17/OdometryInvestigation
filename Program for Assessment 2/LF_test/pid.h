
#ifndef _PID_H
#define _PID_H


float error;
// Class to contain generic PID algorithm.
class PID_c {
  public:
    float p_term;
    float feedback;
    float last_error;
    float i_term;
    float i_sum;
    float d_term;
    float p_gain;
    float i_gain;
    float d_gain;
    unsigned long ms_last_ts;
    // Constructor, must exist.
    PID_c() {

    }

    void initialize( float kp, float ki, float kd) {
      feedback = 0;
      last_error = 0;
      p_term = 0;
      i_term = 0;
      d_term = 0;
      i_sum = 0;

      p_gain = kp;
      i_gain = ki;
      d_gain = kd;

      ms_last_ts = millis();
    }


    void reset() {
      p_term = 0;
      i_term = 0;
      d_term = 0;
      i_sum = 0;
      feedback = 0;
      last_error = 0;
      ms_last_ts = millis();
    }


    float update( float demand, float measurement) {

      float error;
      unsigned long ms_now_ts;
      unsigned long ms_dt;
      float float_dt;
      float diff_error;

      ms_now_ts = millis();
      ms_dt  = ms_now_ts - ms_last_ts;


      ms_last_ts = millis();

      float_dt = (float)ms_dt;

      if (float_dt == 0) return feedback;

      error =  - measurement + demand;

      p_term = p_gain * error;

      i_sum = i_sum + (error * float_dt);
      i_term = i_gain * i_sum;

      diff_error = (error - last_error) / float_dt;
      last_error = error;
      d_term = diff_error * d_gain;


      feedback = p_term + i_term + d_term;

      return feedback;
    }
};



#endif
