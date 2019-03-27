#include "PIDOO.h"

// Prototype to set PID parameters
void PIDOO::set_PID_constants(float kp_in, float ki_in, float kd_in, float I_e_in, float D_in) {
  kp = kp_in;
  ki = ki_in;
  kd = kd_in;
  I_e = I_e_in;
  D = D_in;
}

void PIDOO::set_desired_value(float des_in) {
  des = des_in;
}

void PIDOO::limit_des_val(float lower_lim, float upper_lim) {
  if (des < lower_lim) {
    des = lower_lim;
  }
  else if (des > upper_lim) {
    des = upper_lim;
  }
}

// Prototype to calculate PID outputs
float PIDOO::compute_PID (float current_val, float time_diff) {
  // Compute Error
  float e = des - current_val;
  // Compute Integral
  I_e = I_e + e*time_diff;
  // Compute Derivative
  D = (current_val - old_val)/time_diff;
  // Compute PID Output
  output = e*kp + ki*I_e - kd*D;
  // Update old_yaw
  old_val = current_val;

  return output;
}

