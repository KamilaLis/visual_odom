#include <ros/ros.h>
#include "visual_odom/component_odom.h"

namespace visual_odom
{

ComponentOdom::ComponentOdom(const std::string& subFrom)
{
  ros::NodeHandle n;
  sub_ = n.subscribe(subFrom, 1000, &ComponentOdom::odomCallback, this);
}


void ComponentOdom::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //robot twist
  this->twist_linear_ = msg->twist.twist.linear.x;
  this->twist_angular_ = msg->twist.twist.angular.z;
  //ROS_INFO("twist_angular_: [%f]", twist_angular_);
}

double ComponentOdom::getTwistLinear()
{
  return this->twist_linear_;
}

double ComponentOdom::getTwistAngular()
{
  return this->twist_angular_;
}


}; //namespace


int main(int argc, char **argv)
{

  ros::init(argc, argv, "visual_odom");

  visual_odom::ComponentOdom viso2("/mono_odometer/odometry");
  visual_odom::ComponentOdom base_controller("/elektron/mobile_base_controller/odom");

  ROS_INFO("VISUAL_ODOM STARTED !!");

  ros::spin();
  return 0;
}
