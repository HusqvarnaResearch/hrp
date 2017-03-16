/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#ifndef AUTOMOWER_LEGACY_H
#define AUTOMOWER_LEGACY_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <am_driver/Loop.h>
#include <am_driver/SensorStatus.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Imu.h>
#include <am_driver/WheelEncoder.h>
#include <am_driver/WheelCurrent.h>

#include <sys/select.h>

namespace Husqvarna
{

class AutomowerLegacy
{
public:
    AutomowerLegacy(const ros::NodeHandle& nodeh);
    ~AutomowerLegacy();

    bool setup();
    bool update(ros::Duration dt);

private:
    void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel);
    void modeCallback(const std_msgs::UInt16::ConstPtr& msg);

    bool initAutomowerBoard();
    int twosComp(int val, int bits);
    bool sendNavigation(signed short lv, signed short rv);
    int sendMessage(unsigned char* msg, int len, unsigned char* ansmsg, int maxAnsLength, bool retry);
    void imuResetCallback(const geometry_msgs::Pose::ConstPtr& msg);

    double regulateVelocity(ros::Duration dt, double wanted_vel, double actual_vel);
    void doAccelerationControl(ros::Duration dt);

    // ROS data
    ros::NodeHandle nh;

    ros::Subscriber velocity_sub;
    ros::Subscriber cmd_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber imu_euler_sub;

    ros::Publisher pose_pub;
    ros::Publisher odom_pub;

    ros::WallTime last_command_time;

    ros::Publisher loop_pub;
    ros::Publisher sensorStatus_pub;

    ros::Publisher encoder_pub;
    ros::Publisher current_pub;

    ros::Subscriber imuResetSub;

    tf::TransformBroadcaster br;

    // Control data
    double lin_vel;
    double ang_vel;

    geometry_msgs::PoseStamped robot_pose;

    double xpos, ypos, yaw;
    double wanted_lv, wanted_rv;
    double current_lv, current_rv;
    double last_yaw;

    bool automowerInterfaceInited;
    int leftPulses;
    int rightPulses;
    int lastLeftPulses;
    int lastRightPulses;
    int leftTicks;
    int rightTicks;

    // Parameters
    std::string pSerialPort;

    // Mower parameters (treated as const, that is why capital letters...sorry)
    int WHEEL_SPEED_TIMER_TICK;
    double WHEEL_DIAMETER;
    int WHEEL_PULSES_PER_TURN;
    double WHEEL_METER_PER_TICK;

    // Serial port handle
    int serialFd;

    // Serial port state
    int serialPortState;

    // Automower status
    am_driver::Loop loop;
    am_driver::SensorStatus sensorStatus;
    unsigned short mode;
    am_driver::WheelEncoder encoder;
    am_driver::WheelCurrent wheelCurrent;

    int publishTf;
    int velocityRegulator;

    // "bonnafusing" (i.e. simple fusing)
    double imuOffset;

    // Cutting disc
    bool cuttingDiscOn;
    unsigned char cuttingHeight;
    unsigned char lastCuttingHeight;

    // FINISHED
    bool reqFinished;

    // GEOFENCE
    bool reqFrontOutside;
    bool reqRearOutside;

    uint8_t actionResponse;
    
    bool invertedMotors;
};

typedef boost::shared_ptr<AutomowerLegacy> AutomowerLegacyPtr;
}

#endif
