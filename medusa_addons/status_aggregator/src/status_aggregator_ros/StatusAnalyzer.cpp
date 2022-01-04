/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.

Don't you miss the danger
*/
#include "StatusAnalyzer.h"

/**
 * @brief Construct a new pluginlib export class object
 * 
 */
PLUGINLIB_EXPORT_CLASS(diagnostic_aggregator::StatusAnalyzer, diagnostic_aggregator::Analyzer)

namespace diagnostic_aggregator {

// @.@ Constructor StatusAnalyzer
StatusAnalyzer::StatusAnalyzer()
    : level_(diagnostic_msgs::DiagnosticStatus::OK) {}


// @.@ Helper member function to initialize status/group analyzer
bool StatusAnalyzer::init(const std::string base_path, const ros::NodeHandle &n) {
    
    // +.+ Calls init() from base class
    bool success = AnalyzerGroup::init(base_path, n);
    if (not success)
        return false;

    ros::NodeHandle nh = n;
    status_publisher_ = nh.advertise<std_msgs::Int8>("status", 10);
    service_print_ = nh.advertiseService("status", &StatusAnalyzer::statusPrintService, this);

    return true;
}

// @.@ Helper member function that reports the status of all devices
std::vector<diagnostic_msgs::DiagnosticStatusPtr> StatusAnalyzer::report() {

    // +.+ Calls report() from base class
    std::vector<diagnostic_msgs::DiagnosticStatusPtr> output = AnalyzerGroup::report();

    // +.+ Find group status
    auto it = std::find_if(
        output.begin(), output.end(),
        [&](const diagnostic_msgs::DiagnosticStatusConstPtr &status) {
            return status->name == getPath();
        });

    // +.+ Publish 1 if aggregator has any sensor with error or staling. 0 otherwise
    if ((*it)->level == diagnostic_msgs::DiagnosticStatus::ERROR || (*it)->level == diagnostic_msgs::DiagnosticStatus::STALE) {
        std_msgs::Int8 msg;
        msg.data = 1;
        status_publisher_.publish(msg);
    } else {
        std_msgs::Int8 msg;
        msg.data = 0;
        status_publisher_.publish(msg);
    }

    return output;
}

// @.@ Service to print the status of the devices
bool StatusAnalyzer::statusPrintService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    res.success = true;

    // +.+ Grab diagnostics information
    std::vector<diagnostic_msgs::DiagnosticStatusPtr> output = AnalyzerGroup::report();

    // +.+ Construct message to be printed
    std::string msg;
    for (auto sensor : output) {
        msg += sensor->name + " " + sensor->hardware_id + " " + status_str[sensor->level] + "\n";
    }

    // +.+ Print message
    // ROS_WARN("%s", msg.c_str());
    res.message = msg.c_str();
    return true;
}
} // namespace diagnostic_aggregator