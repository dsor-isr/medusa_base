/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.

Don't you miss the danger
*/
#ifndef STATUS_ANALYZER_H
#define STATUS_ANALYZER_H

#include <diagnostic_aggregator/analyzer_group.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

/**
 * @brief Status levels: 0-Running, 1-Warn, 2-Error, 3-Not Running
 * 
 */
static const char *status_str[] = { "Running", "Warn - Check messages", "Error - Something is wrong ABORT", "Not Running" };

/**
 * @brief 
 * 
 */
namespace diagnostic_aggregator {

/**
 * @brief Class StatusAnalyzer derived from AnalyzerGroup
 * 
 */
class StatusAnalyzer : public AnalyzerGroup {
 public:
  
  /**
   * @brief Construct a new Status Analyzer object
   * 
   */
  StatusAnalyzer();

 /**
   * @brief Initializes StatusAnalyzer from namespace.  
   *
   * @param base_path : Prefix for all analyzers (ex: 'Sensors')
   * @param n : NodeHandle in full namespace
   * @return True if initialization succeed, false if no errors  
   */
  bool init(const std::string base_path, const ros::NodeHandle &n) override;
  
  /**
   * @brief Reports current state, returns vector of formatted status messages
   * 
   * @return std::vector<diagnostic_msgs::DiagnosticStatusPtr> 
   */
  std::vector<diagnostic_msgs::DiagnosticStatusPtr> report() override;

 private:
  ros::Publisher status_publisher_;     ///< Publishes global status 0-everything ok 1-something fishy
  int8_t level_;                        ///< level of the messages, default OK/Running-0
  ros::ServiceServer service_print_;    ///< Service to print the status of the StatusAnalyzer groups (sensors, actuators, power system)
  
  /**
   * @brief 
   * 
   * @param req empty message, just to start service
   * @param res response of the service, a bool and a string
   * @return true 
   * @return false 
   */
  bool statusPrintService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
};

}  // namespace diagnostic_aggregator

#endif  // STATUS_ANALYZER_H