#include <Arduino.h>
#include <inttypes.h>

class PID {
  public:
    // Constructor
    PID () : kp(0.0), ki(0.0), kd(0.0), I_e(0.0), D(0.0), old_val(0.0), des(0.0), output(0.0) {}
    
    // Prototype to set PID parameters
    void set_PID_constants(float kp_in, float ki_in, float kd_in);
    void set_desired_value(float des_in);
    void limit_des_val(float lower_lim, float upper_lim);

    // Prototype to calculate PID outputs
    float compute_PID (float current_val, float time_diff);

  private:
    float kp;
    float ki;
    float kd; 
    float I_e;
    float D;
    float old_val;
    float des;
    float output;
};

