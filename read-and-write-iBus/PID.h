#include <Arduino.h>
#include <inttypes.h>

class PID {
  public:
    // Prototypes to set PID parameters
    void set_x_constants(float kp, float ki, float kd, float I_e, float D);
    void set_y_constants(float kp, float ki, float kd, float I_e, float D);
    void set_z_constants(float kp, float ki, float kd, float I_e, float D);
    void set_phi_constants(float kp, float ki, float kd, float I_e, float D);
    void set_psi_constants(float kp, float ki, float kd, float I_e, float D);
    void set_theta_constants(float kp, float ki, float kd, float I_e, float D);

    // Prototypes to calculate PID outputs
    void compute_required_attitude (float current_psi);
    void compute_x_PID (float current_x, float desired_x, float time_diff);
    void compute_y_PID (float current_y, float desired_y, float time_diff);
    float compute_z_PID (float current_z, float desired_z, float time_diff);
    float compute_phi_PID (float current_phi, float time_diff);
    float compute_theta_PID (float current_theta, float time_diff);
    float compute_psi_PID (float current_psi, float desired_psi, float time_diff);
    

  private:

    static constexpr float max_roll_angle = 8;
    static constexpr float max_tilt_angle = 8;
    // PID Constants
    float kp_phi;
    float ki_phi;
    float kd_phi; 
    float I_e_phi;
    float D_phi;
    float old_phi;
    float desired_phi;
    float phi_output;
    
    float kp_theta;
    float ki_theta;
    float kd_theta;
    float I_e_theta;
    float D_theta;
    float old_theta;
    float desired_theta;
    float theta_output;
    
    float kp_psi;
    float ki_psi;
    float kd_psi;
    float I_e_psi;
    float D_psi;
    float old_psi;
    float psi_output;
    
    float kp_x;
    float ki_x;
    float kd_x; 
    float I_e_x;
    float D_x;
    float old_x;
    float x_output;
    
    float kp_y; 
    float ki_y;
    float kd_y; 
    float I_e_y;
    float D_y;
    float old_y;
    float y_output;
    
    float kp_z; 
    float ki_z;
    float kd_z; 
    float I_e_z;
    float D_z;
    float old_z;
    float z_output;
};

