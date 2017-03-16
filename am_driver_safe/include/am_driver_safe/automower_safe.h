/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#ifndef AUTOMOWER_H
#define AUTOMOWER_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <am_driver/Loop.h>
#include <am_driver/SensorStatus.h>
#include <am_driver/BatteryStatus.h>
#include <am_driver/WheelPower.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Imu.h>
#include <am_driver/WheelEncoder.h>
#include <am_driver/WheelCurrent.h>
#include <am_driver_safe/TifCmd.h>

#include <sys/select.h>


#ifdef __cplusplus
extern "C"
{
#endif

    // To get rid of loads of compiler warnings...
    #pragma GCC diagnostic ignored "-Wwrite-strings"

    #include "hcp/hcp_types.h"
    #include "hcp/hcp_runtime.h"
    #include "hcp/hcp_string.h"
    #include "hcp/hcp_library.h"

    #include "hcp/amg3.h"

    #pragma GCC diagnostic pop

    typedef struct tCodec
    {
        char* path;
        hcp_szStr name;
        hcp_tCodecLibrary* lib;
    } tCodec;

    typedef struct
    {
        hcp_tVector header;
        hcp_tCodec fixed[HCP_MAXSIZE_CODECS];
    } tCodecSet;

#ifdef __cplusplus
}
#endif


namespace Husqvarna
{

class PidRegulator
{
public:
    void Init(double in_p, double in_i, double in_d)
    {
        p = in_p;
        i = in_i;
        d = in_d;

        Restart();
    }

    void Restart()
    {
        pErr = 0.0;
        iErr = 0.0;
        dErr = 0.0;
    }

    double Update(double currentSignal, double wantedSignal)
    {
        double errOld = pErr;
        double err = (wantedSignal - currentSignal);

        pErr = err;
        iErr = iErr + errOld;
        dErr = err - errOld;

        double out = p*pErr + i*iErr + d*dErr;

        //std::cout << "out: " << out << " pErr:" << pErr << " iErr:" << iErr << " dErr:" << dErr << std::endl;

        return out;
    }

private:
    double pErr;
    double iErr;
    double dErr;

    // Gains
    double p;
    double i;
    double d;

};


class AutomowerSafe
{
public:
    AutomowerSafe(const ros::NodeHandle& nodeh, double anUpdateRate);
    ~AutomowerSafe();

    bool setup();
    bool update(ros::Duration dt);

private:
    void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel);
    void modeCallback(const std_msgs::UInt16::ConstPtr& msg);
    std::string resultToString(hcp_tResult result);
    bool initAutomowerBoard();
    bool sendMessage(const char* msg, int len, hcp_tResult& result);
    void imuResetCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void regulateVelocity();
    bool getRealTimeData();
    bool getSensorData();

    bool executeTifCommand(am_driver_safe::TifCmd::Request& req,
                                      am_driver_safe::TifCmd::Response& res);

    double regulatePid(double current_vel, double wanted_vel);
    bool doSerialComTest();
    void stopWheels();

    std::string loadJsonModel(std::string fileName);

    // ROS data
    ros::NodeHandle nh;

    ros::ServiceServer tifCommandService;
    ros::Subscriber velocity_sub;
    ros::Subscriber cmd_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber imu_euler_sub;

    ros::Publisher pose_pub;
    ros::Publisher odom_pub;

    ros::WallTime last_command_time;

    ros::Publisher loop_pub;
    ros::Publisher sensorStatus_pub;
    ros::Publisher batStatus_pub;

    ros::Publisher encoder_pub;
    ros::Publisher current_pub;
    ros::Publisher wheelPower_pub;

    ros::Subscriber imuResetSub;

    tf::TransformBroadcaster br;

    
    double updateRate;
    
    
    // Control data
    double lin_vel;
    double ang_vel;

    geometry_msgs::PoseStamped robot_pose;

    double xpos, ypos, yaw;
    double wanted_lv, wanted_rv;
    double current_lv, current_rv;
    double last_yaw;
    int16_t power_l, power_r;


    bool automowerInterfaceInited;
    int leftPulses;
    int rightPulses;
    int lastLeftPulses;
    int lastRightPulses;
    int leftTicks;
    int rightTicks;

    // Parameters
    std::string pSerialPort;
    bool serialComTest;
    bool serialLog;

    // Mower parameters (treated as const, that is why capital letters...sorry)
    double WHEEL_DIAMETER;
    int WHEEL_PULSES_PER_TURN;
    double WHEEL_METER_PER_TICK;
    double AUTMOWER_WHEEL_BASE_WIDTH;

    // Serial port handle
    int serialFd;

    // Serial port state
    int serialPortState;

    // Automower status
    am_driver::Loop loop;
    am_driver::SensorStatus sensorStatus;
    unsigned short mode;
    unsigned short requestedMode;
    am_driver::WheelEncoder encoder;
    am_driver::WheelCurrent wheelCurrent;
    am_driver::WheelPower wheelPower;

    
    bool active;


    int publishTf;
    int velocityRegulator;

    // "bonnafusing" (i.e. simple fusing)
    double imuOffset;

    // Cutting disc
    bool cuttingDiscOn;
    bool lastCuttingDiscOn;
    unsigned char cuttingHeight;
    unsigned char lastCuttingHeight;

    uint8_t actionResponse;

	bool loopOn;
	bool requestedLoopOn;
	


    // For HCP Library
    hcp_tHost hcpHost;
    hcp_tState* hcpState;
    tCodecSet hcpCodecs;
    std::string jsonFile;
    hcp_Size_t codecId;
    hcp_Int modelId;

    PidRegulator leftWheelPid;
    PidRegulator rightWheelPid;

    ros::Time nextAutomowerInitTime;
    int lastComtestWheelMotorPower;
    double startTime;


    // Battery
    int batteryCheckCounter;
    bool printCharge;
    am_driver::BatteryStatus batteryStatus;

    // Sensor
    int sensorCheckCounter;

    // Loopdata
    int loopSensorCheckCounter;

	
	// Mode and state
	int stateCheckCounter;

    // To keep disc off in random mode
    int sendStopCuttingDiscCounter;
    
    // UserStop
    bool userStop;

};

typedef boost::shared_ptr<AutomowerSafe> AutomowerSafePtr;
}

#endif
