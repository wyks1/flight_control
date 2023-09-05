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


void revOdometryCallback(const nav_msgs::Odometry& odom)
{
  geometry_msgs::PoseStamped msg1;
  msg1.header = odom.header;
  msg1.header.frame_id = "map";
  ros::Time time_now = ros::Time::now();
  static ros::Time last_time;

  tf2::Quaternion quaternion(
                     odom.pose.pose.orientation.x,
                     odom.pose.pose.orientation.y,
                     odom.pose.pose.orientation.z,
                     odom.pose.pose.orientation.w
  );
  tf2::Matrix3x3 matrix(quaternion);
  double uwb_roll, uwb_pitch, t265_yaw;
    matrix.getRPY(uwb_roll, uwb_pitch,t265_yaw);

  t265_x = odom.pose.pose.position.x;
  t265_y = odom.pose.pose.position.y;
  t265_z = odom.pose.pose.position.z;
   
  ROS_INFO("the t265 yaw is %f",t265_yaw);
  cmd.header.stamp = time_now;
  cmd.position.x = t265_x;
  cmd.position.y = t265_y;
  cmd.position.z = (1.00-t265_z)/10+1.00;
  cmd.yaw = t265_yaw;
  if (t265_yaw<0)
    { 
      cmd.yaw = t265_yaw + 3.14/30 ; 
    }
  
  if (t265_yaw>0)
    { 
      cmd.yaw = t265_yaw - 3.14/30 ; 
    }

  if (abs(t265_yaw)<0.25)
    {
       cmd.yaw = 0 ;
    }
  pos_cmd_pub.publish(cmd);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "GetYawBack");
    ros::NodeHandle nh;

    pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 50);

    pos_sub = nh.subscribe("/t265/odom/sample", 1, revOdometryCallback);
    
    ros::spin();

}





