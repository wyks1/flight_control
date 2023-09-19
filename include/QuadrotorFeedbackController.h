#pragma once

#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/VFR_HUD.h>
#include <std_msgs/Bool.h>
#include <Eigen/Dense>
#include "ParamLoad.h"


#define LOOP_FREQUENCY 150
#define PI 3.1415
#define ZERO 0
#define ONE 1

#define OFFBOARD "OFFBOARD"

enum state{
    HOVER,READY,PLANNING,END
};

struct DataCentre{
    bool rc_state;
    ros::Time begin_time;
    Eigen::Vector3d wrapper_last_position_;
    ros::Time last_v_time;
    Eigen::Vector3d wrapper_current_position_, wrapper_current_velocity_, wrapper_current_attitude_, wrapper_current_acc_;
    std::string current_state_;
        
    mavros_msgs::AttitudeTarget thrust_attitude_cmd_;
    std_msgs::Bool is_ready_;
    geometry_msgs::PoseStamped pub_setpoint_position_;
    geometry_msgs::TwistStamped pub_setpoint_velocity_;
    geometry_msgs::Vector3Stamped pub_setpoint_attitude_, pub_euler_attitude_, pub_new_velocity,pub_position_int,pub_velocity_int;
    double yaw;

    double thrust_eval[5];
};

class QuadrotorFeedbackController{
    private:
        // const ros::NodeHandle &nh_;
        //data
        geometry_msgs::PoseStamped position_setpoint_;
        geometry_msgs::TwistStamped velocity_setpoint_;
        geometry_msgs::Vector3Stamped  position_int,velocity_int;
        mavros_msgs::AttitudeTarget thrust_attitude_cmd_;


        std::string current_state_;
        std::string current_status_;
    
        // hover PID params
        double kp_hover_x_, kp_hover_y_, kp_hover_z_, kp_hover_vx_, kp_hover_vy_, kp_hover_vz_;
        double ki_hover_x_, ki_hover_y_, ki_hover_z_, ki_hover_vx_, ki_hover_vy_, ki_hover_vz_;
        double kd_hover_x_, kd_hover_y_, kd_hover_z_, kd_hover_vx_, kd_hover_vy_, kd_hover_vz_;

        //thrust eval
        int eval_ptr_;

        Eigen::Vector3d current_position_, current_velocity_, current_attitude_;
        Eigen::Vector3d position_error_sum_, velocity_error_sum_;
        Eigen::Vector3d position_error_before_, velocity_error_before_;

    public:
        struct DataCentre *data_ptr;
        
        //function
        QuadrotorFeedbackController(geometry_msgs::PoseStamped position_setpoint, struct DataCentre* wrap_data_ptr);
        ~QuadrotorFeedbackController();

        void loadLatestData();
        void reset_error_sum_both_pv();
        //void positionControlFeedback();
        //void velocityControlFeedback();
	void positionPlanningFeedback(geometry_msgs::PoseStamped position_setpoint);
        void velocityPlanningFeedback(double psi_cmd);
        void get_params(const ros::NodeHandle &nh);
};


