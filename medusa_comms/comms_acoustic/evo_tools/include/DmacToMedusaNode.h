/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico 
*/
#ifndef CATKIN_WS_DMACTOMEDUSANODE_H
#define CATKIN_WS_DMACTOMEDUSANODE_H

//some generically useful stuff to include...
#include <string>

#include <ros/ros.h> //ALWAYS need to include this

// ROS messages and stuff
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <auv_msgs/NavigationStatus.h>
#include <dmac/mUSBLFix.h>
#include <medusa_msgs/mUSBLFix.h>
#include <medusa_gimmicks_library/MedusaGimmicks.h>
#include <memory>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <math.h>

class DmacToMedusaNode
{
public:
	// #############################
	// @.@ Constructor
	// #############################
	// "main" will need to instantiate a ROS nodehandle, then pass it to the constructor
	DmacToMedusaNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);

	// #############################
	// @.@ Destructor
	// #############################
	~DmacToMedusaNode();

private:
	// put private member data here; "private" data will only be available to member functions of this class;
	ros::NodeHandle nh_, nh_private_; // we will need this, to pass between "main" and constructor

	// some objects to support subscriber and publishers, these will be set up within the class constructor, hiding the ugly details

	// #####################
	// @.@ Subsctibers
	// #####################
    ros::Subscriber usbl_fix_dmac_sub_;
    ros::Subscriber state_sub_;
	// #####################
	// @.@ Publishers
	// #####################
	// Example: ros::Publisher uref_pub;
    ros::Publisher usbl_fix_medusa_pub_;

	// ####################################################################################################################
	// member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
	// member variables will retain their values even as callbacks come and go
	// ####################################################################################################################

	// +.+ package.xml Parameters from Yaml
	// always p_paraName -> Example: double p_ku;
	std::string p_vehicle_name_;
  // +.+ simulation(false) or real(true)
  bool p_real_{false};
  bool p_fix_type_{false};

  // +.+ installation matrix of the usbl
  std::vector<double> p_installation_matrix_;


    // +.+ topic name as parameters from parameter server
    std::string p_usbl_fix_dmac_topic_, p_usbl_fix_medusa_topic_;

	// +.+ Problem variables
    Eigen::MatrixXd usbl_rot_matrix_ = Eigen::MatrixXd(3,3);
    Eigen::MatrixXd body_rot_matrix_ = Eigen::MatrixXd(3,3);
    double yaw_state_;
    
  	
    // #######################################################################################
	// @.@ Encapsulation the gory details of initializing subscribers, publishers and services
	// #######################################################################################
	void initializeSubscribers();
	void initializePublishers();
	void loadParams();

    // #######################################################################################
    // @.@ Helper functions
    // #######################################################################################

    void buildUSBLRotationMatrix();

	// #######################################################################################
	// @.@ Callbacks declaration
	// #######################################################################################
    void fixCallback(const dmac::mUSBLFix& dmac_usbl_fix_msg);
    void stateCallback(const auv_msgs::NavigationStatus &msg);
};
#endif //CATKIN_WS_DMACTOMEDUSANODE_H
