#include "OffboardWrapper.h"

OffboardWrapper::OffboardWrapper(geometry_msgs::PoseStamped position_setpoint, std::string id, std::string node_id, std::string dataset)
{
  uav_id = id;
  dataset_address = dataset;
  begin_once_flag = 1;
  // Publisher
  m_Publisher.wrapper_local_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>(uav_id + "/mavros/setpoint_position/local", 10);
  m_Publisher.wrapper_vision_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>(uav_id + "/mavros/vision_pose/pose", 10);
  m_Publisher.wrapper_attitude_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>(uav_id + "/mavros/setpoint_raw/attitude", 10);

  m_Publisher.position_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>(node_id + "/position_setpoint", 10);
  if(begin_once_flag){
    m_Publisher.waypoint_begin_pub = nh.advertise<geometry_msgs::PoseStamped>(node_id + "/waypoint_begin_fuel", 10);
  }
  m_Publisher.velocity_setpoint_pub = nh.advertise<geometry_msgs::TwistStamped>(node_id + "/velocity_setpoint", 10);
  m_Publisher.attitude_setpoint_pub = nh.advertise<geometry_msgs::Vector3Stamped>(node_id + "/attitude_setpoint", 10);
  m_Publisher.attitude_cureuler_pub = nh.advertise<geometry_msgs::Vector3Stamped>(node_id + "/attitude_euler", 10);
  m_Publisher.wrapper_status_pub = nh.advertise<std_msgs::Bool>(node_id + "/status", 10);
  m_Publisher.wrapper_new_velocity_pub = nh.advertise<geometry_msgs::Vector3Stamped>(node_id + "/new_velocity", 10);

  m_Publisher.wrapper_position_int_pub = nh.advertise<geometry_msgs::Vector3Stamped>(node_id + "/pid_position_int", 10);
  m_Publisher.wrapper_velocity_int_pub = nh.advertise<geometry_msgs::Vector3Stamped>(node_id + "/pid_velocity_int", 10);

  // subscriber();
  subscriber();
  start_position_setpoint_ = position_setpoint;
  planning_position_setpoint_ = position_setpoint;
  autohover_position_setpoint_.pose.position.x = start_position_setpoint_.pose.position.x;
  autohover_position_setpoint_.pose.position.y = start_position_setpoint_.pose.position.y;
  autohover_position_setpoint_.pose.position.z= start_position_setpoint_.pose.position.z/3;
  waypoint_begin_pub_.pose.position.z = -1;
  double psi_cmd_=0;
  current_status_ = HOVER;
  use_lidar_data = false;
  hover_flag = 1;
  planning_flag = 1;
  end_flag = 1;
  not_achieved_flag = 1;
  
}

OffboardWrapper::~OffboardWrapper()
{
}

void OffboardWrapper::getEndPoint()
{
  ifstream fin(dataset_address);
  string line, word;
  vector<string> row;
  vector<vector<string>> content;
  int row_num = 0, col_num = 0;

  while (getline(fin, line))
  {
    row.clear();
    stringstream str(line);
    col_num = 0;
    while (getline(str, word, ','))
    {
      row.push_back(word);
      content.push_back(row);
      col_num += 1;
    }
    row_num += 1;
  }
  // std::cout << col_num << std::endl;
  end_position_setpoint_.pose.position.x = stof(content[(row_num)*col_num-2][1]);
  end_position_setpoint_.pose.position.y = stof(content[(row_num)*col_num-2][2]);
  end_position_setpoint_.pose.position.z = stof(content[(row_num)*col_num-2][3]);
  fin.close();
}

void OffboardWrapper::isAtSetpoint()
{
  Eigen::Vector3d hp_(start_position_setpoint_.pose.position.x,
                      start_position_setpoint_.pose.position.y,
                      start_position_setpoint_.pose.position.z);
  Eigen::Vector3d dis_ = wrap_data.wrapper_current_position_ - hp_;
  if (dis_.norm() < 0.2)
  {
    if (hover_flag)
    {
      start_hover_t = ros::Time::now();
      hover_flag = 0;
    }
    if ((ros::Time::now() - start_hover_t).toSec() >= 2.5){
        // current_status_ = HOVER; // enter planning
        waypoint_begin_pub_.pose.position.z = 1;
      current_status_ = PLANNING;
      
    }
     
  }
  else
    current_status_ = HOVER;
}

bool OffboardWrapper::isAutoHoverpoint(geometry_msgs::PoseStamped  set_position)
{
  Eigen::Vector3d hp_(set_position.pose.position.x,
                      set_position.pose.position.y,
                      set_position.pose.position.z);
  wrap_data.wrapper_current_position_[2] = wrap_data.lidar_z_position;

  Eigen::Vector3d dis_ = wrap_data.wrapper_current_position_ - hp_;
  ROS_INFO("inAutoHoverpoint");
  if (dis_.norm() < 0.1)
  {
    if (hover_flag)
    {
      start_hover_t = ros::Time::now();
      hover_flag = 0;
    }
    if ((ros::Time::now() - start_hover_t).toSec() >= 2.0){
        // current_status_ = HOVER; // enter planning
      if(set_position.pose.position.z == start_position_setpoint_.pose.position.z){
          ROS_INFO("AutoHover Success");
          current_status_ =  PLANNING;
      }
     return true;
    }
     
  }
  else
    current_status_ = HOVER;
    return false;
}

void OffboardWrapper::topicPublish()
{
  wrap_data.thrust_attitude_cmd_.header.frame_id = "base_footprint";
  wrap_data.thrust_attitude_cmd_.header.stamp = ros::Time::now();
  m_Publisher.wrapper_new_velocity_pub.publish(wrap_data.pub_new_velocity);
  m_Publisher.wrapper_attitude_pub_.publish(wrap_data.thrust_attitude_cmd_);
  
  m_Publisher.position_setpoint_pub.publish(wrap_data.pub_setpoint_position_);
  if (begin_once_flag){
    m_Publisher.waypoint_begin_pub.publish(waypoint_begin_pub_);
    if(waypoint_begin_pub_.pose.position.z>0)
    {
      begin_once_flag=0;
    }
  }
  // std::cout << wrap_data.pub_setpoint_position_ << std::endl;
  m_Publisher.velocity_setpoint_pub.publish(wrap_data.pub_setpoint_velocity_);
  m_Publisher.attitude_setpoint_pub.publish(wrap_data.pub_setpoint_attitude_);
  m_Publisher.attitude_cureuler_pub.publish(wrap_data.pub_euler_attitude_);

  m_Publisher.wrapper_position_int_pub.publish(wrap_data.pub_position_int);
  m_Publisher.wrapper_velocity_int_pub.publish(wrap_data.pub_velocity_int);

  wrap_data.is_ready_.data = current_status_;
  m_Publisher.wrapper_status_pub.publish(wrap_data.is_ready_);

}

void OffboardWrapper::subscriber()
{
  // m_Subscriber.wrapper_state_sub_ = nh.subscribe<mavros_msgs::State>(uav_id + "/mavros/state",
  //                                                                    10,
  //                                                                    &OffboardWrapper::stateCallback,
  //                                                                    this);
  // in real
  m_Subscriber.wrapper_rc_state_sub_ = nh.subscribe<mavros_msgs::VFR_HUD>("/mavros/vfr_hud",
                                                      10, 
                                                      &OffboardWrapper::rc_state_Callback, 
                                                      this);
  /*m_Subscriber.wrapper_vrpn_sub_ = nh.subscribe<nav_msgs::Odometry>("/t265/odom/sample",
                                                      10, 
                                                      &OffboardWrapper::visualCallback, 
                                                      this);*/
  m_Subscriber.wrapper_pos_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/t265/odom/sample", 10));
  m_Subscriber.wrapper_Lidar_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh,"/Lidar_position", 10));
  sync_t265_lidar_.reset(new message_filters::Synchronizer<OffboardWrapper::SyncPolicyT265Lidar>(
      OffboardWrapper::SyncPolicyT265Lidar(100), *m_Subscriber.wrapper_pos_sub_, *m_Subscriber.wrapper_Lidar_sub_));
  sync_t265_lidar_->registerCallback(boost::bind(&OffboardWrapper::visualCallback, this, _1, _2));
  // in simulator
  // m_Subscriber.wrapper_vrpn_sub_ = nh.subscribe<nav_msgs::Odometry>(uav_id + "/mavros/local_position/odom",
  //                                                                   10,
  //                                                                   &OffboardWrapper::visualCallback,
  //                                                                   this);

  // m_Subscriber.wrapper_current_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(uav_id + "/mavros/local_position/pose",
  //                                                                              10,
  //                                                                              &OffboardWrapper::localCallback,
  //                                                                              this);
  //m_Subscriber.wrapper_velocity_sub_ = nh.subscribe<geometry_msgs::TwistStamped>("outer_velocity",
  //                                                                               10,
  //                                                                               &OffboardWrapper::velocityCallback,
  //                                                                               this);
  //m_Subscriber.wrapper_acc_sub_ = nh.subscribe<geometry_msgs::TwistStamped>("outer_acc",
  //                                                                          10,
  //                                                                          &OffboardWrapper::accCallback,
  //                                                                          this);

  m_Subscriber.wrapper_poscmd_sub_= nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd",
                                                               100, 
                                                               &OffboardWrapper::positioncmdCallback, 
                                                               this,
                                                               ros::TransportHints().tcpNoDelay());
  // arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav_id + "/mavros/cmd/arming");
  // set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_id + "/mavros/set_mode");
  // offb_set_mode.request.custom_mode = "OFFBOARD";
  // arm_cmd.request.value = true;

//   if(uav_id == "/uav0")
//     m_Subscriber.wrapper_status_sub = nh.subscribe<std_msgs::Bool>("/offb1_node/status",
//                                                                    10,
//                                                                    &OffboardWrapper::statusCallback,
//                                                                    this);
//   else
//     m_Subscriber.wrapper_status_sub = nh.subscribe<std_msgs::Bool>("/offb_node/status",
//                                                                    10,
//                                                                    &OffboardWrapper::statusCallback,
//                                                                    this);
}

// void OffboardWrapper::stateCallback(const mavros_msgs::State::ConstPtr &msg)
// {
//   wrapper_current_state_ = *msg;
//   wrap_data.current_state_ = wrapper_current_state_.mode;
// }
void OffboardWrapper::rc_state_Callback(const mavros_msgs::VFR_HUD::ConstPtr &msg){
  wrap_data.rc_state = msg->groundspeed;
  //std::cout<<msg->groundspeed<<std::endl;
}

// void OffboardWrapper::localCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//   geometry_msgs::PoseStamped wrapper_current_local;
//   tf::Quaternion rq;
//   wrapper_current_local = *msg;

//   // Vector3d cur_position_(wrapper_current_local.pose.position.x,
//   //                        wrapper_current_local.pose.position.y,
//   //                        wrapper_current_local.pose.position.z);

//   // wrap_data.wrapper_current_position_ = cur_position_;

//   tf::quaternionMsgToTF(wrapper_current_local.pose.orientation, rq);
//   tf::Matrix3x3(rq).getRPY(wrap_data.wrapper_current_attitude_[0],
//                            wrap_data.wrapper_current_attitude_[1],
//                            wrap_data.wrapper_current_attitude_[2]);
// }
// in real
 void OffboardWrapper::visualCallback(const nav_msgs::Odometry::ConstPtr& msg, const geometry_msgs::PoseStampedConstPtr& msg1){
   nav_msgs::Odometry wrapper_current_vrpn_ = *msg;


   tf::Quaternion rq;
   Vector3d cur_position_(wrapper_current_vrpn_.pose.pose.position.x, 
                         wrapper_current_vrpn_.pose.pose.position.y, 
                         wrapper_current_vrpn_.pose.pose.position.z);
                         //msg1->pose.position.z);
  double  Lidar_z_position = msg1->pose.position.z;

   Vector3d cur_velocity_(wrapper_current_vrpn_.twist.twist.linear.x,
                         wrapper_current_vrpn_.twist.twist.linear.y,
                         wrapper_current_vrpn_.twist.twist.linear.z);
  double  Lidar_z_velocity = msg1->pose.orientation.z;
  
                         //msg1->pose.orientation.z);
  /*ROS_INFO("The cur_x is %f",wrapper_not_achieved_flagcurrent_vrpn_.pose.pose.position.x);
  ROS_INFO("The cur_y is %f",wrapper_current_vrpn_.pose.pose.position.y);
  ROS_INFO("The cur_z is %f",wrapper_current_vrpn_.pose.pose.position.z);

  ROS_INFO("The lidar_z is %f",msg1->pose.position.z);*/
  // Vector3d cur_velosity_;
  // cur_velosity_ = (cur_position_ - wrap_data.wrapper_last_position_) / (ros::Time::now() - wrap_data.last_v_time).toNSec();
  // wrap_data.last_v_time = ros::Time::now();
  // wrap_data.wrapper_last_position_ = cur_position_;
  // wrap_data.wrapper_current_velocity_ = cur_velosity_;
  // std::cout << (ros::Time::now() - wrap_data.last_v_time).toNSec() << std::endl;

  wrap_data.wrapper_current_position_ = cur_position_;
  wrap_data.wrapper_current_velocity_ = cur_velocity_;
  wrap_data.lidar_z_position = Lidar_z_position;
  wrap_data.lidar_z_velocity = Lidar_z_velocity;


  tf::quaternionMsgToTF(wrapper_current_vrpn_.pose.pose.orientation, rq);
  tf::Matrix3x3(rq).getRPY(wrap_data.wrapper_current_attitude_[0],
                           wrap_data.wrapper_current_attitude_[1],
                           wrap_data.wrapper_current_attitude_[2]);

  // geometry_msgs::PoseStamped wrapper_vision_poplanning_position_setpoint_s_ = wrapper_current_vrpn_;
  // wrapper_vision_pos_.header.stamp = ros::Time::now();
  // m_Publisher.wrapper_vision_pos_pub_.publish(wrapper_vision_pos_);
}
// in simulator
// void OffboardWrapper::visualCallback(const nav_msgs::Odometry::ConstPtr &msg)
// {
//   nav_msgs::Odometry wrapper_current_vrpn_ = *msg;
//   Vector3d cur_position_(wrapper_current_vrpn_.pose.pose.position.x,
//                          wrapper_current_vrpn_.pose.pose.position.y,
//                          wrapper_current_vrpn_.pose.pose.position.z);

//   wrap_data.wrapper_current_position_ = cur_position_;
// }

void OffboardWrapper::statusCallback(const std_msgs::Bool::ConstPtr& msg)
{
  ready_flag = (*msg).data;
}

void OffboardWrapper::positioncmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd){

  planning_position_setpoint_.pose.position.x = cmd->position.x;
  planning_position_setpoint_.pose.position.y = cmd->position.y;
  planning_position_setpoint_.pose.position.z = cmd->position.z;
  psi_cmd_ = cmd->yaw;

}

void OffboardWrapper::run()
{
  wrap_data.begin_time = ros::Time::now();
  QuadrotorFeedbackController c1(start_position_setpoint_, &wrap_data);
  //QuadrotorAggressiveController c2(&wrap_data);
  //getEndPoint(); // end_position_setpoint_ from this function
  //c2.readCsvData(dataset_address);
  //QuadrotorFeedbackController c3(end_position_setpoint_, &wrap_data);

  ros::Rate rate(LOOP_FREQUENCY);

  while (ros::ok())
  {
    // std::cout << "end_position_setpoint_.pose.position.x: " << end_position_setpoint_.pose.position.x << std::endl;
    // std::cout << "end_position_setpoint_.pose.position.y: " << end_position_setpoint_.pose.position.y << std::endl;
    // std::cout << "end_position_setpoint_.pose.position.z: " << end_position_setpoint_.pose.position.z << std::endl;
    // start_planning_t_ = ros::Time::now();
    // if (wrap_data.current_state_ != OFFBOARD)
    // {
    //   if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    //     ROS_INFO("OFFBOARD enabled");
    // }
    // else
    // {
    //   if (!wrapper_current_state_.armed)
    //   {
    //     if (arming_client.call(arm_cmd) && arm_cmd.response.success)
    //       ROS_INFO("Vehicle armed");
    //   }
    // }

    switch (current_status_)
    {
    case HOVER:
      ROS_INFO("enter hover!!\n");
      if(isAutoHoverpoint(autohover_position_setpoint_)&&not_achieved_flag )
          autohover_position_setpoint_.pose.position.z = autohover_position_setpoint_.pose.position.z + start_position_setpoint_.pose.position.z/3;

      if(autohover_position_setpoint_.pose.position.z == start_position_setpoint_.pose.position.z)
          not_achieved_flag = 0;
  
      c1.loadAutoHoverData();
      if(!wrap_data.rc_state){
        c1.reset_error_sum_both_pv();
        ROS_INFO("RESET I");
      }
      c1.positionPlanningFeedback(autohover_position_setpoint_);
      c1.velocityPlanningFeedback(0);
      start_planning_t_ = ros::Time::now();
      break;

    case READY:
     // ROS_INFO("enter ready!!\n");
      // if(ready_flag) current_status_ = PLANNING;
      current_status_ = PLANNING;
      c1.loadLatestData();
      c1.positionPlanningFeedback(start_position_setpoint_);
      c1.velocityPlanningFeedback(0);
      start_planning_t_ = ros::Time::now();
      break;


    case PLANNING:
      ROS_INFO("enter planning!!\n");
      /*if(begin_once_flag)
          isAtSetpoint();*/

      c1.loadLatestData();
      c1.positionPlanningFeedback(planning_position_setpoint_);
      c1.velocityPlanningFeedback(psi_cmd_);
      break;

      
    }
    topicPublish();
    ros::spinOnce();
    rate.sleep();
  }
}
