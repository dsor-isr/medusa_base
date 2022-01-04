/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico 

Don't you miss the danger
*/
#ifndef CATKIN_WS_MEDUSADIAGNOSTICSNODE_H
#define CATKIN_WS_MEDUSADIAGNOSTICSNODE_H

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>

/**
 * @brief MedusaDiagnostics namespace
 * 
 */
namespace MedusaDiagnostics{

	/**
	 * @brief Set the Diagnosis Msg object (DiagnosticStatus)
	 * 
	 * @param level: 0->OK, 1-> WARN, 2-> ERROR, 3->STALE 
	 * @param name: name of what sensor/node is being diagnosed, ex: /sensors/ + node_name
	 * @param message: Say if it is ok or not 
	 * @param hardware_id: name of the sensor 
	 * @return diagnostic_msgs::DiagnosticStatus 
	 */
	diagnostic_msgs::DiagnosticStatus setDiagnosisMsg(const uint8_t &level, const std::string &name, const std::string &message, const std::string &hardware_id);

	/**
	 * @brief add key values to diagnostic message
	 * 
	 * @param diagnostic_msg  array(DiagnosticArray) of diagnostic_msgs
	 * @param key_name name of what we are diagnosing ex: Temperature, Current, Yaw
	 * @param value value being diagnosed, ex: from sensor
	 * @param index of the diagnostic_msg array
	 */
	void addKeyValue(diagnostic_msgs::DiagnosticArray *diagnostic_msg, const std::string &key_name, const std::string &value, const unsigned int &index);

	/**
	 * @brief  Check lower bound value
	 * 
	 * @tparam T type of the values to be compared (int, double, float)
	 * @param value value being diagnosed, ex: received by the sensor
	 * @param lower_bound value of lower bound defined by the user
	 * @return true if the sensor value is lower than the lower bound
	 * @return false if the sensor value is bigger than the lower bound
	 */
	template <typename T> bool checkLowerBound(const T &value, const T &lower_bound){
		return value < lower_bound ? true : false; 
	}

	/**
	 * @brief  
	 * 
	 * @tparam T type of the values to be compared (int, double, float)
	 * @param value received by the sensor 
	 * @param upper_bound value of the upper bound defined by the user 
	 * @return true if the sensor value is bigger than the upper bound
	 * @return false if the sensor value is lower thant the upper bound
	 */
	template <typename T> bool checkUpperBound(const T &value, const T &upper_bound){
		return value > upper_bound ? true : false;
	}

	/**
	 * @brief Define the level as WARN and change the message in diagnostics 
	 * 
	 * @param diagnostic_msg array(DiagnosticArray) of diagnostic_msgs
	 * @param message to clarify warning
	 * @param index of the diagnostic_msg array
	 */
	void warnLevel(diagnostic_msgs::DiagnosticArray *diagnostic_msg, const std::string &message, const unsigned int &index);

	/**
	 * @brief Define the level as Error and change the message in diagnostics
	 * 
	 * @param diagnostic_msg array(DiagnosticArray) of diagnostic_msgs
	 * @param message to clarify error
	 * @param index of the diagnostic_msg array
	 */
	void errorLevel(diagnostic_msgs::DiagnosticArray *diagnostic_msg, const std::string &message, const unsigned int &index);

};

#endif //CATKIN_WS_MEDUSADIAGNOSTICSNODE_H
