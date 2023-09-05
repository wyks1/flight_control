#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/Corrections.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ros::Publisher pos_cmd_pub;
ros::Subscriber pos_sub;
double t265_x,t265_y,t265_z,t265_yaw;

quadrotor_msgs::PositionCommand cmd;

void cmdCallback(const ros::TimerEvent &e) {
    
    ros::Time time_now = ros::Time::now();
    static ros::Time last_time;
    double cmd_time = time_now.toSec();
    cmd.header.stamp = time_now;
    cmd.position.x = 0;
    cmd.position.y = 0;
    cmd.position.z = 0.5;
    cmd.yaw = sin(cmd_time/10)*3.14;
    pos_cmd_pub.publish(cmd);
    ROS_INFO("x: [%f]", cmd.position.x);
    ROS_INFO("y: [%f]", cmd.position.y);
    ROS_INFO("z: [%f]", cmd.position.z);
    ROS_INFO("yaw: [%f]",  cmd.yaw );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_test");
    ros::NodeHandle nh;

    ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

    pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 50);
    
    ros::spin();

}




