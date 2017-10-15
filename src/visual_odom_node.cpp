

#include <ros/ros.h>
#include "visual_odom/component_odom.h"

namespace visual_odom
{

ComponentOdom::ComponentOdom(const std::string& subFrom, bool cmd):
    buff_angular_(0.0),
    buff_linear_(0.0),
    linear_(0.0),
    angular_(0.0)
{
  ros::NodeHandle n;
  if (cmd) sub_ = n.subscribe(subFrom, 1000, &ComponentOdom::cmdCallback, this);
  else sub_ = n.subscribe(subFrom, 1000, &ComponentOdom::viso2Callback, this);
}

void ComponentOdom::viso2Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  double lin = msg->twist.twist.linear.z;
  if ((lin>=-1 && lin<=-0.001)||(lin>=0.001 && lin<=1)){
    linear_ = lin;
  }

  double ang = -(msg->twist.twist.angular.y);
  if ((ang>=-1 && ang<=-0.001)||(ang>=0.001 && ang<=1))
    angular_ = ang;
    
}

void ComponentOdom::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  linear_ = msg->linear.x;
  angular_ = msg->angular.z;
}

void ComponentOdom::kalmanCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  linear_ = msg->linear.x;
  angular_ = msg->angular.z;
  if (!had_first_callback_) had_first_callback_=true;
}

double ComponentOdom::getTwistLinear()
{
  return linear_;
}

double ComponentOdom::getTwistAngular()
{
  return angular_;
}

double ComponentOdom::getOldLinear()
{
  return buff_linear_;
}

double ComponentOdom::getOldAngular()
{
  return buff_angular_;
}



}; //namespace




std::list<bool> warnings_;

void updateWindow(bool warning)
{
  warnings_.push_back(warning);
  if (warnings_.size()>=10)
    warnings_.pop_front();

  // check number of warnings in window
  int counter=0;
  for (std::list<bool>::iterator it = warnings_.begin(); it != warnings_.end(); it++)
    if(*it==true) ++counter;
  if (counter>=5)
    ROS_ERROR("Velocities are not equal");

}

bool isEqual(double vel, double cmd_vel)
{
  return vel >= cmd_vel-0.1 && vel <= cmd_vel+0.1; 
}

void infoCallback(const viso2_ros::VisoInfo::ConstPtr& msg, 
  boost::shared_ptr<visual_odom::ComponentOdom> viso2, 
  boost::shared_ptr<visual_odom::ComponentOdom> cmd)
{
  if(!msg->got_lost) //is possible to compare
  {
    ROS_INFO(">> I'm here!");
    bool warning=false;

    if(!isEqual(viso2->getTwistLinear(),cmd->getTwistLinear()))
    {
      ROS_WARN("Current linear not equal, checking last one...");
      if(!isEqual(viso2->getTwistLinear(),cmd->getOldLinear()))
      {
        warning = true;
      }
      else ROS_INFO("Fine! Continuing...");

    }

    if(!isEqual(viso2->getTwistAngular(),cmd->getTwistAngular()))
    {
      ROS_WARN("Current angular not equal, checking last one...");
      if(!isEqual(viso2->getTwistAngular(),cmd->getOldAngular()))
      {
        warning = true;
      }
      else
      {
        ROS_INFO("Fine! Continuing...");
      }

    }
    updateWindow(warning);

    // save current value as old one
    cmd->buff_angular_=cmd->getTwistAngular();
    cmd->buff_linear_=cmd->getTwistLinear();
  }
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_odom");

  boost::shared_ptr<visual_odom::ComponentOdom> viso2_ptr(new visual_odom::ComponentOdom("/mono_odometer/odometry", false));
  boost::shared_ptr<visual_odom::ComponentOdom> cmd_vel_ptr(new visual_odom::ComponentOdom("/mux_vel_raw/cmd_vel", true));
  //boost::shared_ptr<visual_odom::ComponentOdom> kalman_ptr(new visual_odom::ComponentOdom("/filtered_vel", false));

  ros::NodeHandle nh;

  ros::Subscriber info_sub = nh.subscribe<viso2_ros::VisoInfo>("/mono_odometer/info", 
                                                                1000, 
                                                                boost::bind(infoCallback, _1, viso2_ptr, cmd_vel_ptr));
  ros::NodeHandle local_nh("~");
  ros::Publisher diagnostic_pub_ = local_nh.advertise<diagnostic_msgs::DiagnosticStatus>("info", 1);

  ROS_INFO("VISUAL_ODOM STARTED !!");

  ros::spin();
  return 0;
}
