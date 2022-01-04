/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico 
*/
#ifndef CATKIN_WS_GNSS2UTMOUTLIER_H
#define CATKIN_WS_GNSS2UTMOUTLIER_H

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <exception>

#include <ros/ros.h>

// ROS messages and stuff
#include <sensor_msgs/NavSatFix.h>
#include <dsor_msgs/Measurement.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>

// Medusa Libraries and msgs
#include <medusa_gimmicks_library/MedusaGimmicks.h>

// TFs
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Third Party Libraries
#include <GeographicLib/UTMUPS.hpp>

class Gnss2UtmOutlier
{
public:
	// #############################
	// @.@ Constructor
	// #############################
	Gnss2UtmOutlier(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);

	// #############################
	// @.@ Destructor
	// #############################
	~Gnss2UtmOutlier();

private:
	ros::NodeHandle nh_, nh_private_;

	// #####################
	// @.@ Subsctibers
	// #####################
    ros::Subscriber gnss_sub_;

	// #####################
	// @.@ Publishers
	// #####################
	// Example: ros::Publisher uref_pub;
	ros::Publisher state_pub_;

  
  dsor_msgs::Measurement utm_;

	// ####################################################################################################################
	// member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
	// member variables will retain their values even as callbacks come and go
	// ####################################################################################################################

	// +.+ package.xml Parameters from Yaml
  double p_default_depth_;
	std::string p_gnss_topic_, p_state_topic_;
  ros::Timer timer_gps_outlier_;
  ros::Timer timer_usbl_back_;
  ros::ServiceServer enable_gps_outlier_srv_;
  ros::ServiceServer enable_usbl_delay_srv_;


	// +.+ Problem variables
    // #######################################################################################
	// @.@ Encapsulation the gory details of initializing subscribers, publishers and services
	// #######################################################################################
	void initializeSubscribers();
  void initializeTimers();
	void initializePublishers();
  void initializeServices();
	void loadParams();

	// #######################################################################################
	// @.@ Callbacks declaration
	// #######################################################################################
	/* 
	* Function Name: gnssBroadcasterCallback
	* Inputs : GNSS/GPS Fix of the vehicle
	* Effects: Converts Lat/Lon fix to UTM and publishes the fix
	* Outputs: ...
	*/	
	void gnssBroadcasterCallback(const sensor_msgs::NavSatFix &msg);
  void timerGPSOutlierCallback(const ros::TimerEvent &event);
  void timerUSBLDelayCallback(const ros::TimerEvent &event);
  bool enableGPSService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool enableUSBLService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
 };
#endif 
