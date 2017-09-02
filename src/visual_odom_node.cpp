#include <ros/ros.h>
#include "visual_odom/component_odom.h"

#include <viso2_ros/VisoInfo.h>

namespace visual_odom
{

ComponentOdom::ComponentOdom(const std::string& subFrom, bool odom)
{
  ros::NodeHandle n;
  if (odom) sub_ = n.subscribe(subFrom, 1000, &ComponentOdom::odomCallback, this);
  else sub_ = n.subscribe(subFrom, 1000, &ComponentOdom::cmdCallback, this);
}

// czy nie da sie przeciazyc tej funkcji? co zrobic by rozroznial typy?
void ComponentOdom::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //robot twist
  this->linear_ = msg->twist.twist.linear.x;
  this->angular_ = msg->twist.twist.angular.z;
  //ROS_INFO("viso2_linear_: [%f]", linear_);
  //ROS_INFO("viso2_angular_: [%f]", angular_);
}

void ComponentOdom::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  this->linear_ = msg->linear.x;
  this->angular_ = msg->angular.z;
  ROS_INFO("cmd_vel_x_: [%f]", this->linear_);
  ROS_INFO("cmd_vel_z_: [%f]", angular_);
}

double ComponentOdom::getTwistLinear()
{
  return this->linear_;
}

double ComponentOdom::getTwistAngular()
{
  return this->angular_;
}


}; //namespace


bool isEqual(double vis_odom, double cmd_vel){
  return vis_odom >= cmd_vel-0.2 && vis_odom <= cmd_vel+0.2; //+/- costam
}

void infoCallback(const viso2_ros::VisoInfo::ConstPtr& msg, 
  boost::shared_ptr<visual_odom::ComponentOdom> viso2, 
  boost::shared_ptr<visual_odom::ComponentOdom> cmd)
{
  if(!msg->got_lost) //is possible to compare
  {
    ROS_INFO("I'm here!");
    ROS_INFO("viso2_linear_: [%f]", viso2->getTwistLinear());
    ROS_INFO("cmd_vel_linear_: [%f]", cmd->getTwistLinear());
   /* if(!isEqual(viso2.getTwistLinear(),cmd.getTwistLinear()))
      ROS_INFO("Linear velocity is not equal");
    if(!isEqual(viso2.getTwistAngular(),cmd.getTwistAngular()))
      ROS_INFO("Angular velocity is not equal");*/
  }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "visual_odom");

// consider shared_ptr : 
  boost::shared_ptr<visual_odom::ComponentOdom> viso2_ptr(new visual_odom::ComponentOdom("/mono_odometer/odometry", true));
  boost::shared_ptr<visual_odom::ComponentOdom> cmd_vel_ptr(new visual_odom::ComponentOdom("/mux_vel_raw/cmd_vel", false));
 // visual_odom::ComponentOdom viso2("/mono_odometer/odometry", true);
  //visual_odom::ComponentOdom cmd_vel("/mux_vel_raw/cmd_vel", false);

  ros::NodeHandle nh;
  ros::Subscriber info_sub = nh.subscribe<viso2_ros::VisoInfo>("/mono_odometer/info", 
                                                                1000, 
                                                                boost::bind(infoCallback, _1, viso2_ptr, cmd_vel_ptr));



  ROS_INFO("VISUAL_ODOM STARTED !!");

  ros::spin();
  return 0;
}
