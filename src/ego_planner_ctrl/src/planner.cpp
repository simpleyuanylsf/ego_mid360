/*****************************************************************************************
 * egoplanner控制器
 * 本代码采用的mavros的速度控制进行跟踪
 * 遥控器先position模式起飞，然后rviz打点，再切offborad模式即可
 ******************************************************************************************/
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

#define VELOCITY2D_CONTROL 0b101111000111 // 设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
//用到给0，其他都是1.

class Planner
{
public:
  Planner();
  void spin();

private:
  // 回调
  void rcCb(const mavros_msgs::RCIn::ConstPtr &msg);
  void stateCb(const mavros_msgs::State::ConstPtr &msg);
  void positionCb(const nav_msgs::Odometry::ConstPtr &msg);
  void targetCb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void twistCb(const quadrotor_msgs::PositionCommand::ConstPtr &msg);

  // 控制更新
  void updateControl();

  ros::NodeHandle nh_;

  ros::Subscriber state_sub_;
  ros::Subscriber rc_sub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber target_sub_;
  ros::Subscriber position_sub_;

  ros::Publisher local_pos_pub_;
  ros::Publisher pubMarker_;

  ros::ServiceClient arming_client_;
  ros::ServiceClient command_client_;
  ros::ServiceClient set_mode_client_;

  ros::Rate rate_;

  visualization_msgs::Marker trackpoint_;

  unsigned short velocity_mask_;
  mavros_msgs::PositionTarget current_goal_;
  mavros_msgs::RCIn rc_;
  int rc_value_;
  int flag_;
  int flag1_;

  nav_msgs::Odometry position_msg_;
  geometry_msgs::PoseStamped target_pos_;
  mavros_msgs::State current_state_;

  float position_x_begin_, position_y_begin_, position_z_begin_, yaw_begin_;
  bool get_first_pos_;
  float position_x_, position_y_, position_z_, current_yaw_;
  float targetpos_x_, targetpos_y_;

  // EGO 信息
  quadrotor_msgs::PositionCommand ego_;
  float ego_pos_x_, ego_pos_y_, ego_pos_z_;
  float ego_vel_x_, ego_vel_y_, ego_vel_z_;
  float ego_a_x_, ego_a_y_, ego_a_z_;
  float ego_yaw_, ego_yaw_rate_;

  bool receive_;
  float pi_;
};

Planner::Planner()
    : nh_(),
      rate_(50.0),
      velocity_mask_(VELOCITY2D_CONTROL),
      rc_value_(0),
      flag_(0),
      flag1_(0),
      position_x_begin_(0.0f),
      position_y_begin_(0.0f),
      position_z_begin_(0.0f),
      yaw_begin_(0.0f),
      get_first_pos_(false),
      position_x_(0.0f),
      position_y_(0.0f),
      position_z_(0.0f),
      current_yaw_(0.0f),
      targetpos_x_(0.0f),
      targetpos_y_(0.0f),
      ego_pos_x_(0.0f),
      ego_pos_y_(0.0f),
      ego_pos_z_(0.0f),
      ego_vel_x_(0.0f),
      ego_vel_y_(0.0f),
      ego_vel_z_(0.0f),
      ego_a_x_(0.0f),
      ego_a_y_(0.0f),
      ego_a_z_(0.0f),
      ego_yaw_(0.0f),
      ego_yaw_rate_(0.0f),
      receive_(false),
      pi_(3.14159265f)
{
  setlocale(LC_ALL, "");
  std::cout << "start" << std::endl;

  state_sub_ =
      nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &Planner::stateCb, this);
  local_pos_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
      "/mavros/setpoint_raw/local", 1);

  pubMarker_ =
      nh_.advertise<visualization_msgs::Marker>("/track_drone_point", 5);

  rc_sub_ = nh_.subscribe<mavros_msgs::RCIn>(
      "/mavros/rc/in", 10, &Planner::rcCb, this);

  arming_client_ =
      nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  command_client_ =
      nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
  set_mode_client_ =
      nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  twist_sub_ = nh_.subscribe<quadrotor_msgs::PositionCommand>(
      "/position_cmd", 10, &Planner::twistCb, this);

  target_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "move_base_simple/goal", 10, &Planner::targetCb, this);

  position_sub_ = nh_.subscribe<nav_msgs::Odometry>(
      "/mavros/local_position/odom", 10, &Planner::positionCb, this);

  current_goal_.coordinate_frame =
      mavros_msgs::PositionTarget::FRAME_BODY_NED;
  current_goal_.type_mask = velocity_mask_;

  ROS_INFO("[Planner] Initialized.");
}

void Planner::rcCb(const mavros_msgs::RCIn::ConstPtr &msg)
{
  rc_ = *msg;
  if (rc_.channels.size() > 4)
  {
    rc_value_ = rc_.channels[4];
  }
}

void Planner::stateCb(const mavros_msgs::State::ConstPtr &msg)
{
  current_state_ = *msg;
}

// read vehicle odometry
void Planner::positionCb(const nav_msgs::Odometry::ConstPtr &msg)
{
  position_msg_ = *msg;
  tf2::Quaternion quat;
  tf2::convert(
      msg->pose.pose.orientation,
      quat); // 把mavros/local_position/pose里的四元数转给tf2::Quaternion quat
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  if (!get_first_pos_)
  {
    position_x_begin_ = position_msg_.pose.pose.position.x;
    position_y_begin_ = position_msg_.pose.pose.position.y;
    position_z_begin_ = position_msg_.pose.pose.position.z;
    yaw_begin_ = yaw;
    get_first_pos_ = true;
  }
  position_x_ = position_msg_.pose.pose.position.x - position_x_begin_;
  position_y_ = position_msg_.pose.pose.position.y - position_y_begin_;
  position_z_ = position_msg_.pose.pose.position.z - position_z_begin_;
  current_yaw_ = yaw;
}

void Planner::targetCb(const geometry_msgs::PoseStamped::ConstPtr &msg) // 读取rviz的航点
{
  target_pos_ = *msg;
  targetpos_x_ = target_pos_.pose.position.x;
  targetpos_y_ = target_pos_.pose.position.y;
}

// 读取ego里的位置速度加速度yaw和yaw-dot信息，其实只需要ego的位置速度和yaw就可以了
void Planner::twistCb(const quadrotor_msgs::PositionCommand::ConstPtr &msg) // ego的回调函数
{
  receive_ = true;
  ego_ = *msg;
  ego_pos_x_ = ego_.position.x;
  ego_pos_y_ = ego_.position.y;
  ego_pos_z_ = ego_.position.z;
  ego_vel_x_ = ego_.velocity.x;
  ego_vel_y_ = ego_.velocity.y;
  ego_vel_z_ = ego_.velocity.z;
  ego_a_x_ = ego_.acceleration.x;
  ego_a_y_ = ego_.acceleration.y;
  ego_a_z_ = ego_.acceleration.z;
  ego_yaw_ = ego_.yaw + yaw_begin_;
  ego_yaw_rate_ = ego_.yaw_dot;
}

void Planner::updateControl()
{
  // take off 1m
  if (!receive_) // 如果没有在rviz上打点，则offboard模式下会保持在0，0，1的高度
  {
    current_goal_.coordinate_frame =
        mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_goal_.header.stamp = ros::Time::now();
    current_goal_.type_mask = velocity_mask_;
    current_goal_.velocity.x = (0 - position_x_) * 1;
    current_goal_.velocity.y = (0 - position_y_) * 1;
    current_goal_.velocity.z = (1 - position_z_) * 1;
    current_goal_.yaw = current_yaw_;
    ROS_INFO("请等待");
  }

  // if receive plan in rviz, the EGO plan information can input mavros and vehicle can auto navigation
  if (receive_) // 触发后进行轨迹跟踪
  {
    float yaw_erro;
    yaw_erro = (ego_yaw_ - current_yaw_);
    (void)yaw_erro; // 目前未使用，仅保持变量存在
    current_goal_.coordinate_frame =
        mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // 选择local系，一定要local系
    current_goal_.header.stamp = ros::Time::now();
    current_goal_.type_mask =
        velocity_mask_; // 这个就算对应的掩码设置，可以看mavros_msgs::PositionTarget消息格式
    current_goal_.velocity.x = (ego_pos_x_ - position_x_) * 2;
    current_goal_.velocity.y = (ego_pos_y_ - position_y_) * 2;
    current_goal_.velocity.z = (ego_pos_z_ - position_z_) * 2;
    current_goal_.yaw = ego_yaw_;
    ROS_INFO("EGO规划速度：vel_x = %.2f",
             std::sqrt(std::pow(current_goal_.velocity.x, 2) +
                       std::pow(current_goal_.velocity.y, 2)));
  }

  local_pos_pub_.publish(current_goal_);
}

void Planner::spin()
{
  while (ros::ok() && !current_state_.connected)
  {
    ros::spinOnce();
    rate_.sleep();
  }

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    current_goal_.coordinate_frame =
        mavros_msgs::PositionTarget::FRAME_BODY_NED;
    local_pos_pub_.publish(current_goal_);
    ros::spinOnce();
    rate_.sleep();
  }

  while (ros::ok() /* &&rc_value_>900&&rc_value_<1150 */)
  {
    
    updateControl();

    ros::spinOnce();
    rate_.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_planner_ctrl");

  Planner planner;
  planner.spin();

  return 0;
}

