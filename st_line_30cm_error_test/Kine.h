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
    float count_per_rot = 358.3;
    float wheel_radius = 16;       // in mm
    float wheel_distance = 40.75;  // l in mm
    float dst_per_rotation = 2 * Pi * wheel_radius;
    float dst_per_count = dst_per_rotation / count_per_rot;
    float x_global = 0;
    float y_global = 0;
    float theta_global = 0;
    float theta_r;
    

    volatile long old_count_e_left = 0;
    volatile long old_count_e_right = 0;

    void updates() {

      float lft_encoder_dst = dst_per_count * (count_l - old_count_e_left);
      float rgt_encoder_dst = dst_per_count * (count_r- old_count_e_right);

      old_count_e_left = count_l;
      old_count_e_right = count_r;

      float x_r = lft_encoder_dst / 2 + rgt_encoder_dst / 2;

      theta_r = lft_encoder_dst / (2 * wheel_distance) - rgt_encoder_dst / (2 * wheel_distance);

      theta_global = theta_global + theta_r;
      float theta_global1 = (theta_global * 180)/Pi;
      x_global = x_global + x_r * cos(theta_global);
      y_global = y_global + x_r * sin(theta_global);

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
