#pragma once

#include <math.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <vector>

/* Include the messages used by publishers */
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

/* Messages used to receive data from the vehicle and from the path */
#include <auv_msgs/NavigationStatus.h>
#include <medusa_msgs/mPStabilizationDebug.h>
#include <medusa_msgs/mPStabilizationTarget.h>

/* Include the Control libraries */
#include "Potes.h"
#include "PathStabilization.h"
#include "States.h"

/* Include the generated services for the path stabilization */
#include "path_stabilization/ResetVT.h"
#include "path_stabilization/SetPS.h"
#include "path_stabilization/StartPS.h"
#include "path_stabilization/StopPS.h"
#include "path_stabilization/UpdateGainsPS.h"

/* Include the waypoint service to use when a mission finishes */
#include "waypoint/sendWpType1.h"
#include <std_srvs/Trigger.h>

/* Define Constants used when starting and stoping the path stabilization */
#define WP_FINISH -3
#define FLAG_IDLE 0
#define FLAG_PS 6

/**
 * @brief     Path Stabilization Node, where the magic happens 
 * @author    André Potes
 * @version   1.0a
 * @date      2022
 * @copyright MIT
 */
class PathStabilizationNode {

  public:

    /**
     * @brief  The constructor of the path stabilization node
     *
     * @param nh  The public ROS node handle
     * @param nh_p The private ROS node handle
     */
    PathStabilizationNode(ros::NodeHandle *nh, ros::NodeHandle *nh_p);

    /**
     * @brief The destructor of the path stabilization node (where the subscribers, publishers
     * and services are terminated)
     */
    ~PathStabilizationNode();

  private:

    /**
     * @brief Pointer to the Path Stabilization algorithm to use 
     */
    PathStabilization *ps_algorithm_{nullptr};

    /**
     * @brief Auxiliar variables to store the current vehicle and target state
     */
    VehicleState vehicle_state_;
    TargetState target_state_;

    /**
     * @brief Auxiliary variables to check whether we have received the first information
     * for intial vehicle and target state
     */
    bool has_received_target_state{false};
    bool has_received_vehicle_state{false};

    /**
     * @brief Use to store the time-elapsed to be used by the controller
     */
    ros::Time prev_time_;

    /**
     * @brief Nodle Handler attributes
     */
    ros::NodeHandle nh_;
    ros::NodeHandle nh_p_;

    /**
     * @brief ROS Services for path stabilization 
     */
    ros::ServiceServer ps_start_srv_;
    ros::ServiceServer ps_stop_srv_;
    ros::ServiceServer ps_update_gains_srv_;

    ros::ServiceServer ps_potes_srv_;

    /* Service to reset the virtual target position */
    ros::ServiceServer ps_reset_vt_srv_;

    /**
     * @brief ROS Services auxiliary to the path stabilization 
     */
    ros::ServiceClient wp_standard_client_;
    ros::ServiceClient dr_reset_client_;         

    /**
     * @brief ROS publishers attributes
     */
    std::vector<ros::Publisher> publishers_;
    ros::Publisher flag_pub_;

    /**
     * @brief ROS subscribers attributes
     */
    std::vector<ros::Subscriber> subscribers_;
    ros::Subscriber vehicle_state_sub_;
    ros::Subscriber target_state_sub_;
    ros::Subscriber flag_sub_;

    /**
     * @brief ROS Timer attributes 
     */
    ros::Timer timer_;

    /**
     * @brief Method to allocate memory for a default PS controller class 
     */
    void getDefaultControllerPotes();

    /**
     * @brief Method to delete the current controller being used
     */
    void deleteCurrentController();

    /**
     * @brief Method to stop the current path_stabilization 
     */
    void stopAlgorithm();

    /**
     * @brief Method to initialize the ROS part
     */
    void initializeSubscribers();
    void initializePublishers();
    void initializeServices();
    void initializeTimer();
    double nodeFrequency();

    /**
     * @brief Method where the logic is located in order to update the control law
     *
     * @param event  A TimerEvent from ros
     */
    void timerIterCallback(const ros::TimerEvent &event);

    /**
     * @brief Callbacks
     */
    void targetStateCallback(const medusa_msgs::mPStabilizationTarget &msg);
    void vehicleStateCallback(const auv_msgs::NavigationStatus &msg);
    void flagCallback(const std_msgs::Int8 &msg);

    /**
     * @brief Services callbacks 
     */
    bool StartPSService(path_stabilization::StartPS::Request &req,
        path_stabilization::StartPS::Response &res);
    bool StopPSService(path_stabilization::StopPS::Request &req,
        path_stabilization::StopPS::Response &res);
    bool UpdateGainsPSService(path_stabilization::UpdateGainsPS::Request &req,
        path_stabilization::UpdateGainsPS::Response &res);

    bool SetPotesService(path_stabilization::SetPS::Request &req,
        path_stabilization::SetPS::Response &res);

    bool ResetVirtualTargetService(path_stabilization::ResetVT::Request &req,
        path_stabilization::ResetVT::Response &res);

    /**
     * @brief Service to send a waypoint when the path stabilization stops
     */
    void sendWaypoint(double value);
    
    /**
     * @brief Service to reset the DeadReckoning position when the path stabilization stops
     */
    void sendResetDeadReckoning();
};
