#ifndef COMPONENTODOM_H
#define COMPONENTODOM_H

#include <iostream>
#include <stdlib.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace visual_odom{

class ComponentOdom
{
public:
    ComponentOdom(const std::string& subFrom);

protected:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    double getTwistLinear();
    double getTwistAngular();

private:
    ros::Subscriber sub_;
    double twist_linear_;
    double twist_angular_;
};

}; //namespace

#endif //COMPONENTODOM_H