/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.

Don't you miss the danger
*/
#include "medusa_diagnostics_library/MedusaDiagnostics.h"

diagnostic_msgs::DiagnosticStatus MedusaDiagnostics::setDiagnosisMsg(const uint8_t &level, const std::string &name, const std::string &message, 
const std::string &hardware_id)
{
  diagnostic_msgs::DiagnosticStatus diag_status;  
  diag_status.level = level;
  diag_status.name  = name;
  diag_status.message = message;
  diag_status.hardware_id = hardware_id;

  return diag_status;
}

void MedusaDiagnostics::addKeyValue(diagnostic_msgs::DiagnosticArray *diagnostic_msg, const std::string &key_name, const std::string &value, const unsigned int &index)
{
  	// +.+ add key values
	diagnostic_msgs::KeyValue key;
	key.key = key_name;
	key.value = value; 
	diagnostic_msg->status[index].values.insert(diagnostic_msg->status[index].values.end(), key);

}

void MedusaDiagnostics::warnLevel(diagnostic_msgs::DiagnosticArray *diagnostic_msg, const std::string &message, const unsigned int &index)
{
  	diagnostic_msg->status[index].level = diagnostic_msgs::DiagnosticStatus::WARN;
		diagnostic_msg->status[index].message = message; 
}

void MedusaDiagnostics::errorLevel(diagnostic_msgs::DiagnosticArray *diagnostic_msg, const std::string &message, const unsigned int &index)
{
  	diagnostic_msg->status[index].level = diagnostic_msgs::DiagnosticStatus::ERROR;
		diagnostic_msg->status[index].message = message; 
}



