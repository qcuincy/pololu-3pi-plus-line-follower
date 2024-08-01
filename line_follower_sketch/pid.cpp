#include <Arduino.h>
#include "encoders.h"
#include "pid.h"

// Class to contain generic PID algorithm.
PID_c::PID_c(float _Kp, float _Ki, float _Kd) {
    maxOutput = 255;
    this -> _Kp = _Kp;
    this -> _Ki = _Ki;
    this -> _Kd = _Kd;
}

void PID_c::initialise() {
    _p_term = 0;
    _I_term = 0;
    _D_term = 0;
    _feedback_signal = 0;
    _prev_time = millis();
    _prev_error = 0;
}

void PID_c::reset() {
    _I_term = 0;
    _prev_time = millis();
    _prev_error = 0;
}

double PID_c::update(double demand, double measurement, float _K_p, float _K_i, float _K_d) {
    _Kp = _Kp == 0 ? _K_p : _Kp;
    _Ki = _Ki == 0 ? _K_i : _Ki;
    _Kd = _Kd == 0 ? _K_d : _Kd;

    double error = measurement - demand; // r(t) - y(t) (inverted)
    _de = error - _prev_error;

    unsigned long current_time = millis();
    _dt = current_time - _prev_time;
    _prev_time = current_time;


    _p_term = _Kp * error; // p = K_{p} * e

    // Check if the time difference is zero
    if (_dt == 0) {
        _I_term += _Ki * error * 0.001; // I = K_{i} * \sum_{i=1}^{k}  (e(t_{i})\Delta t)
        _D_term = 0;
    } else {
        _D_term = _Kd * (_de / _dt); // D = K_{d} * \frac{e(t_{k}) - e(t_{k-1})}{\Delta t}
        if (_feedback_signal < maxOutput && _feedback_signal > -maxOutput) {
            _I_term += _Ki * error * _dt; // I = K_{i} * \sum_{i=1}^{k}  (e(t_{i})\Delta t)
        }
    }
    
    // Check if the sign of the p term and d term are the same
    if ((_p_term > 0 && _D_term > 0) || (_p_term < 0 && _D_term < 0)) {
        _feedback_signal = _p_term + _I_term - _D_term;
    } else {
        _feedback_signal = _p_term + _I_term + _D_term;
    }

    if (_feedback_signal > maxOutput) {
        _feedback_signal = maxOutput;
    } else if (_feedback_signal < -maxOutput) {
        _feedback_signal = -maxOutput;
    }
    
    _prev_error = error;

    return _feedback_signal;
}

void PID_c::updateKp(float new_Kp) {
    _Kp = new_Kp;
}

void PID_c::updateKi(float new_Ki) {
    _Ki = new_Ki;
}

void PID_c::updateKd(float new_Kd) {
    _Kd = new_Kd;
}

void PID_c::updateK(float new_Kp, float new_Ki, float new_Kd) {
    _Kp = new_Kp;
    _Ki = new_Ki;
    _Kd = new_Kd;
}

float PID_c::getDe() {
    return _de;
}

unsigned long PID_c::getDt() {
    return _dt;
}

float PID_c::getPTerm() {
    return _p_term;
}

float PID_c::getITerm() {
    return _I_term;
}

float PID_c::getDTerm() {
    return _D_term;
}
float PID_c::getFeedbackSignal() {
    return _feedback_signal;
}
