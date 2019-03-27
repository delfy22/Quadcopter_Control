#include "PID.h"
#include "math.h"

void PID::set_desired_values(float x_des, float y_des, float z_des, float yaw_des) {
  desired_x = x_des;
  desired_y = y_des;
  desired_z = z_des;
  desired_yaw = yaw_des;
}

// **************************************************************************************************
// Set constants used by PID controllers
void PID::set_x_constants(float kp, float ki, float kd, float I_e, float D) {
  kp_x = kp;
  ki_x = ki;
  kd_x = kd; 
  I_e_x = I_e;
  D_x = D;
}

void PID::set_y_constants(float kp, float ki, float kd, float I_e, float D) {
  kp_y = kp;
  ki_y = ki;
  kd_y = kd; 
  I_e_y = I_e;
  D_y = D;
}

void PID::set_z_constants(float kp, float ki, float kd, float I_e, float D) {
  kp_z = kp;
  ki_z = ki;
  kd_z = kd; 
  I_e_z = I_e;
  D_z = D;
}

void PID::set_xspeed_constants(float kp, float ki, float kd, float I_e, float D) {
  kp_xspeed = kp;
  ki_xspeed = ki;
  kd_xspeed = kd;
  I_e_xs = I_e;
  D_xs = D;
}

void PID::set_yspeed_constants(float kp, float ki, float kd, float I_e, float D) {
  kp_yspeed = kp;
  ki_yspeed = ki;
  kd_yspeed = kd;
  I_e_ys = I_e;
  D_ys = D;
}

void PID::set_zspeed_constants(float kp, float ki, float kd, float I_e, float D) {
  kp_zspeed = kp;
  ki_zspeed = ki;
  kd_zspeed = kd;
  I_e_zs = I_e;
  D_zs = D;
}

// **************************************************************************************************
// pitch and roll conversion 
// Must be called after x and y speed controllers. Accounts for drone's yaw rotation.
float PID::compute_desired_pitch (float current_psi) {
  // Calculate required roll and pitch to reach desired x and y
  float desired_theta = cos(current_psi)*xspeed_output - sin(current_psi)*yspeed_output;

  // Apply limits to roll and pitch
  if (desired_theta > max_tilt_angle) 
    desired_theta = max_tilt_angle;
  else if (desired_theta < -max_tilt_angle)
   desired_theta = -max_tilt_angle;

  return desired_theta;
}
float PID::compute_desired_roll (float current_psi) {
  // Calculate required roll and pitch to reach desired x and y
  float desired_phi = sin(current_psi)*xspeed_output + cos(current_psi)*yspeed_output;

  if (desired_phi > max_roll_angle) 
    desired_phi = max_roll_angle;
  else if (desired_phi < -max_roll_angle)
   desired_phi = -max_roll_angle;

  return desired_phi;
}

// **************************************************************************************************
// Position controllers

//float PID::compute_yaw_PID (float current_yaw, float time_diff) {
//  // Compute Error
//  float e_yaw = desired_yaw - current_yaw;
//  // Compute Integral
//  I_e_yaw = I_e_yaw + e_yaw*time_diff;
//  // Compute Derivative
//  D_yaw = (current_yaw - old_yaw)/time_diff;
//  // Compute PID Output
//  yaw_output = e_yaw*kp_yaw + ki_yaw*I_e_yaw - kd_yaw*D_yaw;
//  // Update old_yaw
//  old_yaw = current_yaw;
//
//  return yaw_output;
//}

float PID::compute_x_PID (float current_x, float time_diff) {
  // Compute Error
  float e_x = desired_x - current_x;
  // Compute Integral
  I_e_x = I_e_x + e_x*time_diff;
  // Compute Derivative
  D_x = (current_x - old_x)/time_diff;
  // Compute PID Output
  x_output = e_x*kp_x + ki_x*I_e_x - kd_x*D_x;
  // Update old_x
  old_x = current_x;

  return x_output;
}

float PID::compute_y_PID (float current_y, float time_diff) {
  // Compute Error
  float e_y = desired_y - current_y;
  // Compute Integral
  I_e_y = I_e_y + e_y*time_diff;
  // Compute Derivative
  D_y = (current_y - old_y)/time_diff;
  // Compute PID Output
  y_output = -(e_y*kp_y + ki_y*I_e_y - kd_y*D_y);
  // Update old_y
  old_y = current_y;

  return y_output;
}

float PID::compute_z_PID (float current_z, float time_diff) {
  // Compute Error
  float e_z = desired_z - current_z;
  // Compute Integral
  I_e_z = I_e_z + e_z*time_diff;
  // Compute Derivative
  D_z = (current_z - old_z)/time_diff;
  // Compute PID Output
  z_output = e_z*kp_z + ki_z*I_e_z - kd_z*D_z;
  // Update old_z
  old_z = current_z;

  // Return the PID output
  return z_output;
}

// **************************************************************************************************
// Speed controllers

float PID::compute_xspeed_PID (float current_xspeed, float desired_xspeed, float time_diff) {
  // Limit speed
  if (desired_xspeed > max_horiz_speed)
    desired_xspeed = max_horiz_speed;
  else if (desired_xspeed < -max_horiz_speed)
    desired_xspeed = -max_horiz_speed;
  
  // Compute Error
  float e_xs = desired_xspeed - current_xspeed;
  // Compute Integral
  I_e_xs = I_e_xs + e_xs*time_diff;
  // Compute Derivative
  D_xs = (current_xspeed - old_xspeed)/time_diff;
  // Compute PID Output
  xspeed_output = e_xs*kp_xspeed + ki_xspeed*I_e_xs - kd_xspeed*D_xs;
  // Update old_y
  old_xspeed = current_xspeed;
  

  // Return the PID output
  return xspeed_output;
}

float PID::compute_yspeed_PID (float current_yspeed, float desired_yspeed, float time_diff) {
  // Limit speed
  if (desired_yspeed > max_horiz_speed)
    desired_yspeed = max_horiz_speed;
  else if (desired_yspeed < -max_horiz_speed)
    desired_yspeed = -max_horiz_speed;
  
  // Compute Error
  float e_ys = desired_yspeed - current_yspeed;
  // Compute Integral
  I_e_ys = I_e_ys + e_ys*time_diff;
  // Compute Derivative
  D_ys = (current_yspeed - old_yspeed)/time_diff;
  // Compute PID Output
  yspeed_output = e_ys*kp_yspeed + ki_yspeed*I_e_ys - kd_yspeed*D_ys;
  // Update old_y
  old_yspeed = current_yspeed;

  // Return the PID output
  return yspeed_output;
}

float PID::compute_zspeed_PID (float current_zspeed, float desired_zspeed, float time_diff) {
  
  // Limit speed
  if (zspeed_output > max_vert_speed)
    zspeed_output = max_vert_speed;
  else if (zspeed_output < -max_vert_speed)
    zspeed_output = -max_vert_speed;
  
  // Compute Error
  float e_zs = desired_zspeed - current_zspeed;
  // Compute Integral
  I_e_zs = I_e_zs + e_zs*time_diff;
  // Compute Derivative
  D_zs = (current_zspeed - old_zspeed)/time_diff;
  // Compute PID Output
  zspeed_output = e_zs*kp_zspeed + ki_zspeed*I_e_zs - kd_zspeed*D_zs;
  // Update old_y
  old_zspeed = current_zspeed;

  // Return the PID output
  return zspeed_output;
}


