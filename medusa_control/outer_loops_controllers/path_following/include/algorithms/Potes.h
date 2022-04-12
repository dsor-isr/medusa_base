#pragma once

#include "PathFollowing.h"
#include "dsor_paths/SpawnCircle2D.h"
#include "dsor_paths/ResetPath.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tuple>
#include <Eigen/Geometry>

/**
 * @brief Path following algorithm used to track a target around a circular path center in it.
 * This algorithm support:
 *    Controls:
 *      - surge
 *      - sway
 *      - heave
 *      - roll rate
 *      - pitch rate
 *      - yaw rate
 *      - virtual target velocity (gamma dot)
 *    Supports Cooperative Path Following - False
 *    Contains Currents Observers - False
 *
 * @author    Andre Potes
 * @author    Francisco Rego
 * @author    Marcelo Jacinto
 * @version   1.0a
 * @date      2021
 * @copyright MIT
 */
class Potes : public PathFollowing {

public:

    /**
     * @brief  Constructor method for the Path Following class
     *position
     * @param kp Positive proportional linear velocity control scalar gain. Represents the transition steepness in saturation function
     * @param lambda_p Positive linear velocity controller gain. Represents the max/min limits of the saturation function
     * @param k_gamma Positive parameter gain for virtual particle controller.
     * @param k_gamma_e Positive parameter gain for virtual particle controller.
     * @param rho_gamma_e Positive parameter gain for virtual particle controller.
     * @param sigma_m Positive parameter for attitude control. Denotes the maximum distance in which the change in the desired yaw angle occurs.
     * @param kpsi Positive parameter for attitude control. Represents the change steepness curve.
     * @param Kp_omega Positive proportional definite gain matrix for attitude control.
     * @param Q Arbitrary matrix such that all singular values are distinct.
     * @param circle_radius Positive parameter related with the circle radius of the path.
     * @param surge_pub The ROS surge publisher
     * @param sway_pub The ROS sway publisher
     * @param heave_pub The ROS sway publisher
     * @param roll_rate_pub The ROS sway publisher
     * @param pitch_rate_pub The ROS yaw publisher
     * @param yaw_rate_pub The ROS yaw publisher
     * @param rabbit_pub The ROS rabbit publisher
     */
    Potes(double kp, double lambda_p, double k_gamma, double k_gamma_e, double rho_gamma_e,
          double k_psi, double sigma_m, Eigen::Vector3d &Kp_omega, Eigen::Matrix3d &Q, double circle_radius,
          ros::Publisher surge_pub, ros::Publisher sway_pub, ros::Publisher heave_pub, 
          ros::Publisher roll_rate_pub, ros::Publisher pitch_rate_pub, ros::Publisher yaw_rate_pub,
          ros::Publisher rabbit_pub);

    /**
        * @brief Method that receives a message from ROS with the center point of circular path in the inertial frame
     */
    void updateMissionLocation(const geometry_msgs::Point &msg);

    /**
     * @brief Method that receives a message from ROS with the target velocity
     */
    void updateTargetPosition(const geometry_msgs::Point &msg);
    /**
     * @brief Method that receives a message from ROS with the target velocity
     */
    void updateTargetVelocity(const geometry_msgs::Twist &msg);


    /**
     * @brief  Method to update the gains, given a vector of doubles
     *
     * @param gains A vector of gains to update in the controller
     * The default order is: kp, lambda_p, k_gamma, k_gamma_e, sigma_gamma_e, kpsi, sigma_m, Kp_omega
     *
     * @return a boolean which represents the success of the operation
     */
    bool setPFGains(std::vector<double> gains) override;

    /**
     * @brief  Method that implements the path following control law
     *
     * @param dt The time step between last call and current call (in seconds)
     */
    void callPFController(double dt) override;

    /**
     * @brief  Method to publish_private the data from the path following
     */
    void publish_private() override;

    /**
     * @brief  Method used in the first run to do initial setup
     */
    void start() override;

    /**
     * @brief  Method used to check whether we reached the end of the algorithm or not
     *
     * @return The success of the operation
     */
    bool stop() override;

    /**
     * @brief  Method used to reset the algorithm control parameters
     * when running the algorithm more than once
     *
     * @return  Whether the reset was made successfully or not
     */
    bool reset() override;

    /**
     * @brief Method to reset the virtual target of the vehicle (gamma)
     * to a pre-specified value. Not all controllers need this (example: Samson, Fossen which use the closest point)
     *
     * @return Whether the reset was made successfully or not
     */
    bool resetVirtualTarget(float value) override;

protected:

    /**
     * @brief Method that applies a saturation to vector s
     * @param s 3D vector
     */
    Eigen::Vector3d sigma(Eigen::Vector3d &s);

    /**
     * @brief Method that given a vector of euler angles (roll, pitch, yaw) generates rotation matrix
     *
     * @param angles vector of angles in radians
     * @return Rotation Matrix
     */
    Eigen::Matrix3d computeRotationMatrix(Eigen::Vector3d &angles);
    Eigen::Matrix3d computeRotationMatrix(double roll, double pitch, double yaw);

    /**
     * @brief Method that given a vector of parameters computes the Skew Symmetric matrix
     *
     * @param s 3D vector
     * @return Skew Symmetric Matrix
     */
    Eigen::Matrix3d computeSkewSymetric(Eigen::Vector3d &s);

    /**
     * @brief Method that returns a tuple with the necessary references for the path following and attitude controller
     *
     * @param p Position of vehicle in inertial frame
     * @return Tuple
     */
    std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix3d, Eigen::Matrix3d, Eigen::Vector3d, Eigen::Vector3d> getReferences(Eigen::Vector3d &p);

private:

    /**
     * @brief Controller parameters for the  path stabilization algorithm (position and angle)
     */
    double kp_{0.0}; 
    double lambda_p_{0.0};
    Eigen::Vector3d Kp_omega_{0.0, 0.0, 0.0};
    Eigen::Matrix3d Q_ = Eigen::MatrixXd::Zero(3, 3);
    double k_gamma_{0.0}; // gamma (proportional gain)
    double k_gamma_e_{0.0}; // gamma (sigmoid tighten gain)
    double rho_gamma_e_{0.0}; // gamma (maximum distance to virtual target) 

    /**
     * @brief Controller guidance parameters
     * 
     */
    double k_psi_{0.0}; // maximum distance to diver when vehicle shoudl point towards to desired mission location
    double sigma_m_{0.0}; // speed of change of heading reference (sigmoid tighteness)

    bool has_received_target_pos_{false};
    bool has_received_target_vel_{false};
    bool has_received_mission_{false};

    /**
     * @brief The safety radius path between diver and robot
     */
    double circle_radius_{2.0};

    /**
     * @brief The desired references to publish
     */
    double desired_surge_{0.0};
    double desired_sway_{0.0};
    double desired_heave_{0.0};
    double desired_roll_rate_{0.0};
    double desired_pitch_rate_{0.0};
    double desired_yaw_rate_{0.0};

    /**
     * @brief The references for the target and mission
     */
    Eigen::Vector3d target_vel_{0.0, 0.0, 0.0};
    Eigen::Vector3d target_pos_{0.0, 0.0, 0.0};
    Eigen::Vector3d mission_pos_{0.0, 0.0, 0.0};

    /**
     * @brief The values for the dynamics of the gamma (virtual rabbit)
     */
    double gamma_ref_dot_{0.0};
    double gamma_ref_{0.0};
    double gamma_dot_{0.0};
    double gamma_{0.0};

    /**
     * @brief ROS publishers to publish the data
     */
    ros::Publisher surge_pub_;
    ros::Publisher sway_pub_;
    ros::Publisher heave_pub_;

    ros::Publisher roll_rate_pub_;
    ros::Publisher pitch_rate_pub_;
    ros::Publisher yaw_rate_pub_;

    ros::Publisher rabbit_pub_;

    /**
     * @brief ROS service to reset the path and to ask for a circular path
     */
    ros::ServiceClient reset_path_client_;
    ros::ServiceClient spawn_circle_client_;
};

