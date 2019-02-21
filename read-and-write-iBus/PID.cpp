#include "PID.h"
#include "math.h"

#define PI 3.14159265

// Set constants used by PID controller
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

void PID::set_phi_constants(float kp, float ki, float kd, float I_e, float D)
{
  kp_phi = kp;
  ki_phi = ki;
  kd_phi = kd; 
  I_e_phi = I_e;
  D_phi = D;
}

void PID::set_psi_constants(float kp, float ki, float kd, float I_e, float D)
{
  kp_psi = kp;
  ki_psi = ki;
  kd_psi = kd; 
  I_e_psi = I_e;
  D_psi = D;
}

void PID::set_theta_constants(float kp, float ki, float kd, float I_e, float D)
{
  kp_theta = kp;
  ki_theta = ki;
  kd_theta = kd; 
  I_e_theta = I_e;
  D_theta = D;
}

// Must be called after x and y controllers, and before roll and pitch (phi and theta) controllers. Accounts for drone's yaw rotation.
void PID::compute_required_attitude (float current_psi) {
  // Calculate required roll and pitch to reach desired x and y
  desired_theta = cos(current_psi)*x_output - sin(current_psi)*y_output;
  desired_phi = sin(current_psi)*x_output + cos(current_psi)*y_output;

  // Apply limits to roll and pitch
  if (desired_theta > max_tilt_angle) 
    desired_theta = max_tilt_angle;
  else if (desired_theta < -max_tilt_angle)
     desired_theta = -max_tilt_angle;

  if (desired_phi > max_roll_angle) 
    desired_phi = max_roll_angle;
  else if (desired_phi < -max_roll_angle)
     desired_phi = -max_roll_angle;
}

// PID controllers
void PID::compute_x_PID (float current_x, float desired_x, float time_diff) {
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
}

void PID::compute_y_PID (float current_y, float desired_y, float time_diff) {
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
}

float PID::compute_z_PID (float current_z, float desired_z, float time_diff) {
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

float PID::compute_phi_PID (float current_phi, float time_diff) {
  // Compute Error
  float e_phi = desired_phi - current_phi;
  // Compute Integral
  I_e_phi = I_e_phi + e_phi*time_diff;
  // Compute Derivative
  D_phi = (current_phi - old_phi)/time_diff;
  // Compute PID Output
  phi_output = e_phi*kp_phi + ki_phi*I_e_phi - kd_phi*D_phi;
  // Update old_phi
  old_phi = current_phi;

  // Return the PID output
  return phi_output;
}

float PID::compute_theta_PID (float current_theta, float time_diff) {
  // Compute Error
  float e_theta = desired_theta - current_theta;
  // Compute Integral
  I_e_theta = I_e_theta + e_theta*time_diff;
  // Compute Derivative
  D_theta = (current_theta - old_theta)/time_diff;
  // Compute PID Output
  theta_output = e_theta*kp_theta + ki_theta*I_e_theta - kd_theta*D_theta;
  // Update old_theta
  old_theta = current_theta;

  // Return the PID output
  return theta_output;
}

// Used by the yaw (psi) controller as drone may rotate by 2PI and angle must then be reset to 0
float wraparound_angle (float angle) {
  // Wraparound angle
    while (angle >= 2*PI) {
        angle = angle - 2*PI;
    }
    while (angle < 0) {
        angle = angle + 2*PI;
    }
    return angle;
}

float PID::compute_psi_PID (float current_psi, float desired_psi, float time_diff) {
  desired_psi = wraparound_angle(desired_psi);
  current_psi = wraparound_angle(current_psi);
  
  // Compute Error
  float e_psi = desired_psi - current_psi;

  // Use the smallest possible magnitude of error, ensures the least movement is required to reach a position
  if (e_psi < -PI) // derived from ((e_psi + 2*PI)^2 < e_psi^2) 
      e_psi += 2*PI;
  else if (e_psi > PI) // derived from ((e_psi - 2*PI)^2 < e_psi^2) 
      e_psi -= 2*PI;
  
  // Compute Integral
  I_e_psi = I_e_psi + e_psi*time_diff;
  
  // Compute Derivative - ensure differential is smallest magnitude possible
  float psi_diff = current_psi - old_psi;
  
  if (psi_diff < - PI)
    D_psi = (psi_diff + 2*PI)/time_diff;
  else if (psi_diff > PI)
    D_psi = (psi_diff - 2*PI)/time_diff;
  else
    D_psi = (psi_diff)/time_diff;

  // Compute PID Output
  psi_output = e_psi*kp_psi + ki_psi*I_e_psi - kd_psi*D_psi;
  // Update old_psi
  old_psi = current_psi;

  // Return the PID output
  return psi_output;
} 




