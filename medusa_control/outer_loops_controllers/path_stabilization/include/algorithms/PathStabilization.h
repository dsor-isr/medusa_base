  #pragma once

#include <Eigen/Core>
#include <vector>

/* Auxiliary definitions for the path state and vehicle state */
#include "States.h"

/* ROS includes for publishing the values and setting the mode of operation */
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <medusa_msgs/mPStabilizationDebug.h>
#include <medusa_msgs/mPStabilizationRefs.h>

/* Contains auxiliary functions for angle wrap-up and others */
#include <medusa_gimmicks_library/MedusaGimmicks.h>
#include <dsor_utils/rotations.hpp>
#include <cmath>

/**
 * @brief  A Base class to update the path stabilization law 
 *
 * @author    André Potes
 * @version   1.0a
 * @date      2021
 * @copyright MIT
 */
class PathStabilization {
  
  public:

   
    /**
     * @brief Virtual destructor for the abstract pathstabilization class
     */
    virtual ~PathStabilization();

    /**
     * @brief  Method to update the path stabilization control law
     *
     * @param dt The time diference between the current and previous call (in seconds)
     */
    virtual void callPSController(double dt) = 0;
    
    /**
     * @brief  Method to publish the data given by the algorithm
     */
    void publish();
    
    virtual void publish_private() = 0;

    /**
     * @brief  Method used to setup the algorithm in the first iteration
     */
    virtual void start() = 0;

    /**
     * @brief  Method used to check whether we have reached the end of the path stabilization
     * algorithm or not. This method will be called in every iteration of the algorithm,
     * and when it return true, the algorithm will stop
     *
     * @return  A boolean that represents whether we have reached the end
     */
    virtual bool stop() = 0;

    /**
     * @brief  Method used to reset the algorithm control parameters 
     * when running the algorithm more than once
     *
     * @return  Whether the reset was made successfully or not
     */
    virtual bool reset() = 0;

    /**
     * @brief Method to reset the virtual target of the vehicle (gamma)
     * to a pre-specified value. Not all controllers need this (example: Samson, Fossen which use the closest point)
     * 
     * @return Whether the reset was made successfully or not
     */
    virtual bool resetVirtualTarget(float value);

    /**
     * @brief Method to reset the virtual target of the vehicle (gamma)
     * to zero. Not all controllers need this.
     * 
     * This method calls the resetVirtualTarget(float value) method which can be overriden by each pf controller
     * @return Whether the reset was made successfully or not
     */
    bool resetVirtualTarget();

    /** 
     * @brief  Receives a vector of gains that should be mapped to the specific controller
     * gains.
     *
     * This method must be implemented by each Path Following class
     *
     * @param gains A vector of gains for the controller
     */
    virtual bool setPSGains(std::vector<double> gains) = 0;

    /** 
     * @brief  Method to update the vehicle state used by the controller 
     *
     * @param vehicle_state A structure with the current state of the vehicle
     */
    void UpdateVehicleState(const VehicleState &vehicle_state);

    /**
     * @brief  Method to update the path state used by the controller 
     *
     * @param path_state A structure with the current state of the path
     */
    void UpdateTargetState(const TargetState &target_state);

    /**
     * @brief Method to set common publishers
     *
     *
    */
    void setPStabilizationDebugPublisher(const ros::Publisher &pstabilization_debug_pub) {pstabilization_debug_pub_ = pstabilization_debug_pub;};
    
  protected:
   
    /**
     * @brief Variable to store the state of the vehicle 
     */
    VehicleState vehicle_state_;
    
    /** 
     * @brief Variable to store the state of the target 
     */
    TargetState target_state_;  
    
    /** 
     * @brief Message variable for debugging path stabilization error variables
     */
    PStabilizationDebug pstabilization_debug_;  

    ros::Publisher pstabilization_debug_pub_; 
    
    /**
     * @brief Auxiliar method to smooth out the angle to be used by path 
     * stabilization algorithms 
     */
    double algConvert(double alg_new, double alg_old, double alg_out_old);
};


