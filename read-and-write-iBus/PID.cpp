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
void PID::set_x_constants(float kp, float ki, float kd, float I_e, float D)
{
  kp_x = kp;
  ki_x = ki;
  kd_x = kd; 
  I_e_x = I_e;
  D_x = D;
}

void PID::set_y_constants(float kp, float ki, float kd, float I_e, float D)
{
  kp_y = kp;
  ki_y = ki;
  kd_y = kd; 
  I_e_y = I_e;
  D_y = D;
}

void PID::set_z_constants(float kp, float ki, float kd, float I_e, float D)
{
  kp_z = kp;
  ki_z = ki;
  kd_z = kd; 
  I_e_z = I_e;
  D_z = D;
}

void PID::set_xspeed_constant(float kp)
{
  kp_xspeed = kp;
}

void PID::set_yspeed_constant(float kp)
{
  kp_yspeed = kp;
}

void PID::set_zspeed_constant(float kp)
{
  kp_zspeed = kp;
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
  float e_xspeed = desired_xspeed - current_xspeed;
  // Compute PID Output
  xspeed_output = e_xspeed*kp_xspeed;

//  Serial.print("xspeedError = "); Serial.println(e_xspeed);

  // Return the PID output
  return xspeed_output;
}

float PID::compute_yspeed_PID (float current_yspeed, float desired_yspeed, float time_diff) {
  // Compute Error
  float e_yspeed = desired_yspeed - current_yspeed;
  // Compute PID Output
  yspeed_output = e_yspeed*kp_yspeed;
  
  // Limit speed
  if (yspeed_output > max_horiz_speed)
    yspeed_output = max_horiz_speed;
  else if (yspeed_output < -max_horiz_speed)
    yspeed_output = -max_horiz_speed;

  // Return the PID output
  return yspeed_output;
}

float PID::compute_zspeed_PID (float current_zspeed, float desired_zspeed, float time_diff) {
  // Compute Error
  float e_zspeed = desired_zspeed - current_zspeed;
  // Compute PID Output
  zspeed_output = e_zspeed*kp_zspeed;

  // Limit speed
  if (zspeed_output > max_vert_speed)
    zspeed_output = max_vert_speed;
  else if (zspeed_output < -max_vert_speed)
    zspeed_output = -max_vert_speed;

  // Return the PID output
  return zspeed_output;
}


