#include <ros/ros.h>
#include "visual_odom/component_odom.h"

#include <viso2_ros/VisoInfo.h>

namespace visual_odom
{

ComponentOdom::ComponentOdom(const std::string& subFrom, bool odom):
    buff_angular_(0.0),
    buff_linear_(0.0)
{
  ros::NodeHandle n;
  if (odom) sub_ = n.subscribe(subFrom, 1000, &ComponentOdom::odomCallback, this);
  else sub_ = n.subscribe(subFrom, 1000, &ComponentOdom::cmdCallback, this);
}

void ComponentOdom::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //robot twist
  double lin = msg->twist.twist.linear.z;
  if ((lin>=-1 && lin<=-0.001)||(lin>=0.001 && lin<=1)){
    this->linear_ = lin;
  }

  double ang = -(msg->twist.twist.angular.y);
  if ((ang>=-1 && ang<=-0.001)||(ang>=0.001 && ang<=1))
    this->angular_ = ang;
    
}

void ComponentOdom::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  this->linear_ = msg->linear.x;
  this->angular_ = msg->angular.z;
}

double ComponentOdom::getTwistLinear()
{
  return this->linear_;
}

double ComponentOdom::getTwistAngular()
{
  return this->angular_;
}

double ComponentOdom::getOldLinear()
{
  return this->buff_linear_;
}

double ComponentOdom::getOldAngular()
{
  return this->buff_angular_;
}

}; //namespace



bool isEqual(double vis_odom, double cmd_vel)
{
  return vis_odom >= cmd_vel-0.1 && vis_odom <= cmd_vel+0.1; 



}

void infoCallback(const viso2_ros::VisoInfo::ConstPtr& msg, 
  boost::shared_ptr<visual_odom::ComponentOdom> viso2, 
  boost::shared_ptr<visual_odom::ComponentOdom> cmd)
{
  if(!msg->got_lost) //is possible to compare
  {
    ROS_INFO(">> I'm here!");
    ROS_INFO("viso_z_: [%f]",viso2->getTwistLinear());
    ROS_INFO("cmd_vel_linear_: [%f]", cmd->getTwistLinear());
    ROS_INFO("viso_ang_y:[%f]",viso2->getTwistAngular());
    ROS_INFO("cmd_vel_angular:[%f]", cmd->getTwistAngular());
    ROS_INFO("cmd_angular_last:[%f]", cmd->getOldAngular());

    if(!isEqual(viso2->getTwistLinear(),cmd->getTwistLinear()))
    {
      ROS_WARN("Current linear not equal, checking last one...");
      if(!isEqual(viso2->getTwistLinear(),cmd->getOldLinear()))
      {
        ROS_ERROR("Linear velocity is not equal");
        ROS_INFO("viso_z_: [%f]",viso2->getTwistLinear());
        ROS_INFO("cmd_vel_linear_: [%f]", cmd->getTwistLinear());
      }
      else ROS_INFO("Fine! Continuing...");

    }

    if(!isEqual(viso2->getTwistAngular(),cmd->getTwistAngular()))
    {
      ROS_WARN("Current angular not equal, checking last one...");
      if(!isEqual(viso2->getTwistAngular(),cmd->getOldAngular()))
      {
        ROS_ERROR("Angular velocity is not equal");
        ROS_INFO("viso_ang_y:[%f]",viso2->getTwistAngular());
        ROS_INFO("cmd_vel_angular:[%f]", cmd->getOldAngular());
      }
      else
      {
        ROS_INFO("Fine! Continuing...");
      }

    }
    // save current value as old one
    cmd->buff_angular_=cmd->getTwistAngular();
    cmd->buff_linear_=cmd->getTwistLinear();
  }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "visual_odom");

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
