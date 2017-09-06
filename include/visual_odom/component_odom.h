#ifndef COMPONENTODOM_H
#define COMPONENTODOM_H

#include <iostream>
#include <stdlib.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

namespace visual_odom{

class ComponentOdom
{
public:
    ComponentOdom(const std::string& subFrom, bool odom);
    double getTwistLinear();
    double getTwistAngular();
    double lin_y_;
    double lin_z_;

protected:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);


private:
    ros::Subscriber sub_;
    double linear_;
    double angular_;
    bool alive_ = false;
};

}; //namespace

#endif //COMPONENTODOM_H