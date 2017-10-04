#ifndef COMPONENTODOM_H
#define COMPONENTODOM_H

#include <iostream>
#include <stdlib.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <viso2_ros/VisoInfo.h>

#include <boost/thread/mutex.hpp>

namespace visual_odom{

class ComponentOdom
{
public:
    ComponentOdom(const std::string& subFrom, bool cmd);
    double getTwistLinear();
    double getTwistAngular();
    double getOldLinear();
    double getOldAngular();
    double buff_linear_;
    double buff_angular_;
    bool had_first_callback_;
    //void updateWindow(bool warning);

protected:
    void viso2Callback(const nav_msgs::Odometry::ConstPtr& msg);
    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void kalmanCallback(const geometry_msgs::Twist::ConstPtr& msg);


private:
    ros::Subscriber sub_;
    double linear_;
    double angular_;
    //std::list<bool> warnings_;
    boost::mutex mutex_;

};

}; //namespace

#endif //COMPONENTODOM_H