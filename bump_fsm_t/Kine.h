#ifndef _KINE_H
#define _KINE_H

int XI = 0;
int YI = 0;
float Xt;
float Yt;
float Ti = 0;
float Tt;
float Xr;
float Yr = 0;
float Tr;
float rad = 16; //mm
float l = 40.75; // mm
float c_p_r = 358.6;
float circum = 100.53; //mm
float d_p_c = circum / c_p_r; //mm
float Hypotenuse;
//  #define d_p_c  0.28

class Kinematics {
  public:

    Kinematics() {

    }

    void updates() {
      float left_disp = d_p_c * count_l;
      float right_disp = d_p_c * count_r;

      Xr = (left_disp + right_disp) / 2;
      Tr = (left_disp - right_disp) / (2 * l);

      Xt = XI + (Xr * cos(Tr));
      Yt = YI + (Xr * sin(Tr));
      Hypotenuse = sqrt((Xt * Xt) + (Yt * Yt));
    }

};



#endif
