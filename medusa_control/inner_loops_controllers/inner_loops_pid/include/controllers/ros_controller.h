#ifndef __ROS_CONTROLLER__
#define __ROS_CONTROLLER__

#include "pid_controller.h"
#include <medusa_gimmicks_library/MedusaGimmicks.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <medusa_msgs/mPidDebug.h>

/**
 * @brief  ROS implementation of the innerloops controllers. Based on a desired reference computes the force or torque to be applied.
 */
class RosController {
public:
  /**
   * @brief  Constructor of a innerloop controller.
   *
   * @param nh  ROS nodehandle to read parameters and subscribe to relevant
   * topics
   * @param controller_name  Controller name (variable being controlled)
   * @param refCallback_topic  Topic name
   * @param state  Pointer to state variable being controlled
   * @param force_or_torque  Pointer to force or torque output
   * @param frequency Frequency of controller sampling rate
   */
  RosController(ros::NodeHandle &nh, std::string controller_name,
                std::string refCallback_topic, double *state,
                double *force_or_torque, double frequency);

  /**
   * @brief  Constructor of a innerloop controller
   *
   * @param nh  ROS nodehandle to read parameters and subscribe to relevant
   * topics
   * @param controller_name  Controller name (variable being controlled)
   * @param refCallback_topic  Topic name
   * @param state  Pointer to state variable being controlled
   * @param state_dot  Pointer to the derivative of the state variable being
   * controlled
   * @param force_or_torque  Pointer to force or torque output
   * @param frequency Frequency of controller sampling rate
   */
  RosController(ros::NodeHandle &nh, std::string controller_name,
                std::string refCallback_topic, double *state, double *state_dot,
                double *force_or_torque, double frequency);
  /**
   * @brief  Core function. Computes the PID output
   *
   * @return  The force or torque that result from the PID computation
   */
  virtual double computeCommand();

  /**
   * @brief  Setter function for the circular units flag
   *
   * @param flag true for controllers using angles, false otherwise
   */
  void setCircularUnits(const bool &flag) { circular_units_ = flag; }
 
  /**
   * @brief Set the Positive Output object
   * 
   * @param flag 
   */
  void setPositiveOutput(const bool &flag) { positive_output_ = flag; }

  /**
   * @brief Get the Controller Name object
   * 
   */
  std::string getControllerName() const {return controller_name_;}

  /**
   * @brief Set the Feedforwar gains P I D object
   * 
   * @param ff Feedforward gain 
   * @param ff_d_gain Feedforwad gain (linear drag)
   * @param ff_dd_gain Feedforwad gain (quadratic drag)
   */
  void setFFGainsPID(const float &kp, const float &ki, const float &kd) {pid_c_->setGains(kp, ki, kd);}

  /**
   * @brief Set the Gains P I D object
   * 
   * @param kp Proportional gain 
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  void setGainsPID(const float &kp, const float &ki, const float &kd) {pid_c_->setGains(kp, ki, kd);}

  void setLimitBoundsPID(const float &max_out, const float &min_out) { pid_c_->setLimitBounds(max_out, min_out);}

protected:
  /**
   * @brief  Callback function. Saturates the value if boundaries exist
   *
   * @param msg  Float64 value of the variable being controlled
   */
  void refCallback(const std_msgs::Float64 &ptr);

  /**
   * @brief  Initialize function. Reads the parameters, creates a pid controller
   * if any gain is different from zero and subscribes to the relevant topic.
   *
   * @param nh  Nodehandle to read parameters and subscribe to relevant
   * topics
   * @param controller_name  Controller name (variable being controlled)
   * @param refCallback_topic  Topic name
   */
  void init(ros::NodeHandle &nh, std::string controller_name,
            std::string refCallback_topic);
  /**
   * @brief  Check if the reference is new
   *
   * @return  True if valid, false otherwise
   */
  virtual bool validRef();

  ros::Duration timeout_ref_;   // inverval of time where a reference is valid
  std::string controller_name_; // string of the variable being controlled (for
                                // reading parameters purpose)
  double ref_value_;     // reference value of the variable being controlled
  double max_ref_value_; // maximum value of the reference being controlled
  double min_ref_value_; // minimum value of the reference being controlled
  ros::Time ref_time_;   // timestamp of the reference
  ros::Time last_cmd_;   // last controller call
  bool debug_;           // flag to check wheter to output or not pid internal information 
  medusa_msgs::mPidDebug debug_msg_; // msg to publish debug information

  // pointers to state values
  double *state_ptr_;

  // pointer to correspondig force or torque
  double *force_or_torque_ptr_;

  // frequency of the controller sampling rate
  double frequency_;

  bool circular_units_;  // for angles
  bool positive_output_; // positive output or switch sign

  // handlers
  PID_Controller *pid_c_;
  ros::Subscriber ros_sub_;
  ros::Publisher debug_pub_;
};

#endif /* ifndef __ROS_CONTROLLER__*/
