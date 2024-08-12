#ifndef _KINE_H
#define _KINE_H

float Xt;
float Yt;
float Tt;

volatile long old_l = 0;
volatile long old_r = 0;
float Hypotenuse;
//  #define d_p_c  0.28


// Class to track robot position.
class Kinematics{
  public:

    // Constructor, must exist.
    Kinematics() {
    }
    const float Pi = 3.141593;
    float c_p_r = 358.3;
    float W_r = 16;       // in mm (Wheel Radius)
    float W_D = 40.75;  // l in mm (Wheel Distance)
    float d_p_r = 2 * Pi * W_r;  // distance per rotation
    float d_p_c = d_p_r / c_p_r; // distance per count
    float x_global = 0;
    float y_global = 0;
    float theta_global = 0;
    float t_r;
    

    volatile long old_count_e_left = 0;
    volatile long old_count_e_right = 0;

    void updates() {

      float l_d = d_p_c * (count_l - old_count_e_left);
      float r_d = d_p_c * (count_r- old_count_e_right);

      old_count_e_left = count_l;
      old_count_e_right = count_r;

      float x_r = l_d / 2 + r_d / 2;

     t_r = l_d / (2 * W_D) - r_d / (2 * W_D);

      theta_global = theta_global + t_r;
      float theta_global1 = (theta_global * 180)/Pi;
      theta_global1 = limitangle(theta_global1);
      float R_t_g = (theta_global1 * Pi)/ 180;
      x_global = x_global + x_r * cos(R_t_g);
      y_global = y_global + x_r * sin(R_t_g);

      // Setting global variables
      Xt = x_global;
      Yt = y_global;
      Tt = theta_global1;
    }

    float limitangle( float x) {
      if (x > 360) {
        x = fmod(x, 360);
      }
      return fabs(x);
    }

    float C_error () {
      float c_d = count_l - count_r;
      float p_v = 10;
      float pwm = c_d * p_v;

      return c_d;

    }
};



#endif
