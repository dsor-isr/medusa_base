#include "pid_controller.h"

PID_Controller::PID_Controller() : p_gain_(0), i_gain_(0), d_gain_(0) {
  reset();
  disable = true;
}

PID_Controller::PID_Controller(float Kp, float Ki, float Kd, float max_error,
                               float max_out)
    : p_gain_(Kp), i_gain_(Ki), d_gain_(Kd), max_error_(max_error),
      min_error_(-max_error), max_out_(max_out), min_out_(-max_out) {
  reset();
  disable = true;
}

PID_Controller::PID_Controller(float Kp, float Ki, float Kd, float max_error,
                               float max_out, float min_error, float min_out)
    : p_gain_(Kp), i_gain_(Ki), d_gain_(Kd), max_error_(max_error),
      min_error_(min_error), max_out_(max_out), min_out_(min_out) {
  reset();
  disable = true;
}

float PID_Controller::computeCommand(float error_p, float duration) {
  // Don't return nothing if controller is disabled
  if (disable || duration < 0.05 || duration > 0.2)
    return 0.0;

  float error = sat(error_p, min_out_, max_out_);
  integral_ += error * duration;

  // Compute PID Terms
  float pTerm = p_gain_ * error;
  float iTerm = i_gain_ * integral_;
  float dTerm = d_gain_ * (error - pre_error_) / duration;

  float out = pTerm + iTerm + dTerm;

  // Saturate output
  if (out > max_out_) {
    out = max_out_;
    integral_ -= error * duration;
  } else if (out < min_out_) {
    out = min_out_;
    integral_ -= error * duration;
  }

  pre_error_ = error;

  return out;
}

void PID_Controller::reset() {
  integral_ = 0;
  pre_error_ = 0;
}

void PID_Controller::setGains(const float &kp, const float &ki,
                              const float &kd) {
  p_gain_ = kp;
  i_gain_ = ki;
  d_gain_ = kd;
}

void PID_Controller::setLimitBounds(const float &max_out,
                                    const float &min_out) {
  max_out_ = max_out;
  min_out_ = min_out;
}

std::vector<double> PID_Controller::getGains() const {
  return std::vector<double>{p_gain_, i_gain_, d_gain_};
}

std::vector<double> PID_Controller::getLimitBounds() const {
  return std::vector<double>{max_out_, min_out_};
}

float PID_Controller::sat(float u, float low, float high) {
  if (u < low)
    return low;
  if (u > high)
    return high;
  return u;
}
