#pragma once

#include "QuadrotorFeedbackController.h"
#include "QuadrotorAggressiveController.h"
#include <vector>
#include <fstream>
#include <iostream>
#include <time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

class OffboardWrapper{
    private:       
        ros::NodeHandle nh;
        geometry_msgs::PoseStamped start_position_setpoint_, end_position_setpoint_,planning_position_setpoint_,autohover_position_setpoint_,waypoint_begin_pub_;
        ros::Time start_planning_t_, start_hover_t;

        // std::string current_status_;
        std::string uav_id;
        std::string dataset_address;
        int current_status_;
        double psi_cmd_;
        //flag
        bool hover_flag;
        bool planning_flag;
        bool end_flag;
        bool ready_flag;
        bool use_lidar_data;
        bool not_achieved_flag;
        bool begin_once_flag;

        void isAtSetpoint();
        bool isAutoHoverpoint(geometry_msgs::PoseStamped  set_position);
        void topicPublish();
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,
                                                          geometry_msgs::PoseStamped> SyncPolicyT265Lidar;
        typedef shared_ptr<message_filters::Synchronizer<SyncPolicyT265Lidar>> SynchronizerT265Lidar;

        SynchronizerT265Lidar sync_t265_lidar_;

    public:
        OffboardWrapper(geometry_msgs::PoseStamped position_setpoint, std::string id, std::string node_id, std::string dataset);
        ~OffboardWrapper();

        mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd;
        ros::ServiceClient set_mode_client;
        ros::ServiceClient arming_client; 

        mavros_msgs::State wrapper_current_state_;

        struct SubsciberWrapper{
            ros::Subscriber wrapper_state_sub_;   
            ros::Subscriber wrapper_current_sub_;
            ros::Subscriber wrapper_vrpn_sub_;
            ros::Subscriber wrapper_lidar_sub_;
            shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> wrapper_pos_sub_;
            shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> wrapper_Lidar_sub_;
            ros::Subscriber wrapper_velocity_sub_;
            ros::Subscriber wrapper_status_sub;

            ros::Subscriber wrapper_acc_sub_;
            ros::Subscriber wrapper_rc_state_sub_;
	    ros::Subscriber wrapper_poscmd_sub_;

        }m_Subscriber;

        struct PublisherWrapper{
            ros::Publisher wrapper_local_pos_pub_;
            ros::Publisher wrapper_attitude_pub_;
            ros::Publisher wrapper_vision_pos_pub_;
            ros::Publisher wrapper_status_pub;  
                      
            ros::Publisher position_setpoint_pub;
            ros::Publisher velocity_setpoint_pub;
            ros::Publisher attitude_setpoint_pub;
            ros::Publisher attitude_cureuler_pub;
            ros::Publisher wrapper_new_velocity_pub;
            ros::Publisher waypoint_begin_pub ;

            ros::Publisher wrapper_position_int_pub;
            ros::Publisher wrapper_velocity_int_pub;
        }m_Publisher;

        struct DataCentre wrap_data;
        
        void subscriber();
        void run();
        void rc_state_Callback(const mavros_msgs::VFR_HUD::ConstPtr& msg);
        void stateCallback(const mavros_msgs::State::ConstPtr& msg);
        //void visualCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void visualCallback(const nav_msgs::Odometry::ConstPtr& msg);//, const geometry_msgs::PoseStampedConstPtr& msg1);
        void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
        void accCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
        void localCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void statusCallback(const std_msgs::Bool::ConstPtr& msg);
	void positioncmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd);
        void getEndPoint();
};

