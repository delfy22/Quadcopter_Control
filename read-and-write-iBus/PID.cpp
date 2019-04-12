#include "PID.h"

// Prototype to set PID parameters
void PID::set_PID_constants(float kp_in, float ki_in, float kd_in) {
  kp = kp_in;
  ki = ki_in;
  kd = kd_in;
  output = 0;
  old_val = 0;
}

void PID::set_desired_value(float des_in) {
  des = des_in;
}

void PID::limit_des_val(float lower_lim, float upper_lim) {
  if (des < lower_lim) {
    des = lower_lim;
  }
  else if (des > upper_lim) {
    des = upper_lim;
  }
}

void PID::reset_integral () {
  I_e = 0;
  D = 0;
}

void PID::saturate_integral (bool sat, float sat_lim) {
  saturate_int = sat;
  sat_limit = sat_lim;
}

float PID::get_PID_val (uint8_t sel) {
  switch (sel) {
    case 0: 
            return e;
            break;
    case 1: 
            return I_e;
            break;
    case 2: 
            return D;
            break;
    default:  
            return 0;
            break;                        
  }
}

// Prototype to calculate PID outputs
float PID::compute_PID (float current_val, float time_diff) {
  // Compute Error
  e = des - current_val;
  // Compute Integral
  I_e = I_e + e*time_diff;

  if (saturate_int) {
    float lower_lim = -sat_limit;
    I_e = constrain(I_e, lower_lim, sat_limit);
  }
  
  // Compute Derivative
  D = (current_val - old_val)/time_diff;
  // Compute PID Output
  output = e*kp + ki*I_e - kd*D;
  // Update old_yaw
  old_val = current_val;

  return output;
}

