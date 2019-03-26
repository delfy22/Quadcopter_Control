#include <Arduino.h>
#include <inttypes.h>

class PID {
  public:
    // Prototype to set and get desired values
    void set_desired_values(float x_des, float y_des, float z_des, float yaw_des);
    
    // Prototypes to set PID parameters
    void set_x_constants(float kp, float ki, float kd, float I_e, float D);
    void set_y_constants(float kp, float ki, float kd, float I_e, float D);
    void set_z_constants(float kp, float ki, float kd, float I_e, float D);
    void set_xspeed_constants(float kp, float ki, float kd, float I_e, float D);
    void set_yspeed_constants(float kp, float ki, float kd, float I_e, float D);
    void set_zspeed_constants(float kp, float ki, float kd, float I_e, float D);

    // Prototypes to convert linear to angular
    float compute_desired_pitch (float current_psi);
    float compute_desired_roll (float current_psi);

    // Prototypes to calculate PID outputs
    float compute_yaw_PID (float current_yaw, float time_diff);
    float compute_x_PID (float current_x, float time_diff);
    float compute_y_PID (float current_y, float time_diff);
    float compute_z_PID (float current_z, float time_diff);
    float compute_xspeed_PID (float current_xspeed, float desired_xspeed, float time_diff);
    float compute_yspeed_PID (float current_yspeed, float desired_yspeed, float time_diff);
    float compute_zspeed_PID (float current_zspeed, float desired_zspeed, float time_diff);
    

  private:

    static constexpr float max_roll_angle = 8;
    static constexpr float max_tilt_angle = 8;
    static constexpr float max_horiz_speed = 0.2; // Limit x and y speed to 0.01ms
    static constexpr float max_vert_speed = 0.2; // Limit z speed to 0.01ms

    float desired_x;
    float desired_y;
    float desired_z;
    float desired_yaw;
    
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

    float kp_xspeed;
    float ki_xspeed;
    float kd_xspeed;
    float I_e_xs;
    float D_xs;
    float old_xspeed;
    float xspeed_output;

    float kp_yspeed;
    float ki_yspeed;
    float kd_yspeed;
    float I_e_ys;
    float D_ys;
    float old_yspeed;
    float yspeed_output;

    float kp_zspeed;
    float ki_zspeed;
    float kd_zspeed;
    float I_e_zs;
    float D_zs;
    float old_zspeed;
    float zspeed_output;
};

