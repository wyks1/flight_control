#include <fstream>
#include <cmath>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nodelet/nodelet.h>
#include <vector>
#include <quadrotor_msgs/Corrections.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <quadrotor_msgs/SO3Command.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>

using namespace std;
using namespace Eigen;
#define PI acos(-1)

vector<Vector3d> wp_list;
Vector3d target_pt;
nav_msgs::Path wp_get;
ros::Subscriber position_cmd_sub_;
ros::Publisher camera_pos;
Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
double des_yaw_, des_yaw_dot_;


void position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd) {
  des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
  kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);
  
  des_yaw_ = cmd->yaw;
  des_yaw_dot_ = cmd->yaw_dot;
  ROS_INFO("x: [%f]", des_pos_(0));
  ROS_INFO("y: [%f]", des_pos_(1));
  ROS_INFO("z: [%f]", des_pos_(2));
  ROS_INFO("yaw: [%f]",  des_yaw_ );
}


int main(int argc, char **argv)
{
    ros::init(argc, argv,"Sendcmd");
    ros::NodeHandle nh;
    ros::Subscriber cmd_sub = nh.subscribe("/planning/pos_cmd", 100, &position_cmd_callback, ros::TransportHints().tcpNoDelay());
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);
    ros::spin();   
}


