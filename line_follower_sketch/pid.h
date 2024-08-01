#include <Arduino.h>
#include "encoders.h"

#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  private:
    double _p_term, _I_term, _D_term;
    double _feedback_signal;
    float _Kp, _Ki, _Kd;
    unsigned long _dt, _prev_time;
    double _de, _prev_error;
    float maxOutput;

  public:
    // Constructor, must exist.
    PID_c(float _Kp = 0, float _Ki = 0, float _Kd = 0);
    void initialise();
    void reset();
    double update(double demand, double measurement, float _K_p = 0, float _K_i = 0, float _K_d = 0);
    void updateKp(float new_Kp);
    void updateKi(float new_Ki);
    void updateKd(float new_Kd);
    void updateK(float new_Kp, float new_Ki, float new_Kd);
    float getDe();
    unsigned long getDt();
    float getPTerm();
    float getITerm();
    float getDTerm();
    float getFeedbackSignal();
};

#endif
