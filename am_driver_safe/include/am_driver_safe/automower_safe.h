/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *  	   Kent Askenmalm
 *
 */

#ifndef AUTOMOWER_SAFE_H
#define AUTOMOWER_SAFE_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
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
#include <am_driver/MotorFeedback.h>
#include <am_driver/MotorFeedbackDiffDrive.h>
#include <sensor_msgs/NavSatFix.h>

#include <sys/select.h>


#include <hq_decision_making/hq_FSM.h>
#include <hq_decision_making/hq_ROSTask.h>
#include <hq_decision_making/hq_DecisionMaking.h>




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


#define	AM_STATE_UNDEFINED     0x0
#define	AM_STATE_IDLE          0x1
#define	AM_STATE_INIT          0x2
#define	AM_STATE_MANUAL        0x3
#define	AM_STATE_RANDOM        0x4
#define	AM_STATE_PARK          0x5



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
    AutomowerSafe(const ros::NodeHandle& nodeh, decision_making::RosEventQueue* eq);
    ~AutomowerSafe();

    bool setup();
    bool update(ros::Duration dt);
    
    decision_making::RosEventQueue* eventQueue;

    bool m_regulatingActive;
    


	void pauseMower();
	void startMower();
	void setParkMode();
	void setAutoMode();
	void cutDiscHandling();
	void cutDiscOff();
    void stopWheels();
	void loopDetectionHandling();
	void cuttingHeightHandling();
	
    void newControlMainState(int aNewState);
    
    int GetUpdateRate();
	

private:
    void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel);
    void powerCallback(const am_driver::WheelPower::ConstPtr& power);
    void modeCallback(const std_msgs::UInt16::ConstPtr& msg);
    
    std::string resultToString(hcp_tResult result);
    bool initAutomowerBoard();
    bool sendMessage(const char* msg, int len, hcp_tResult& result);
    void imuResetCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void regulateVelocity();
    void setPower();
    void sendWheelPower(double power_left,
                        double power_right);
    bool getEncoderData();
    bool getWheelData();
    bool getSensorData();
    bool getPitchAndRoll();
    bool getGPSData();
    bool getStateData();
    bool getSensorStatus();
    bool getLoopData();
    bool getBatteryData();

    bool isTimeOut(ros::Duration elapsedTime, double frequency);

    bool executeTifCommand(am_driver_safe::TifCmd::Request& req,
                                      am_driver_safe::TifCmd::Response& res);

    double regulatePid(double current_vel, double wanted_vel);
    bool doSerialComTest();
    void handleCollisionInjections(ros::Duration dt);

    std::string loadJsonModel(std::string fileName);

    // ROS data
    ros::NodeHandle nh;

    ros::ServiceServer tifCommandService;
    ros::Subscriber velocity_sub;
    ros::Subscriber power_sub;
    ros::Subscriber cmd_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber imu_euler_sub;

    ros::Publisher pose_pub;
    ros::Publisher odom_pub;
    ros::Publisher euler_pub;


    ros::Publisher loop_pub;
    ros::Publisher sensorStatus_pub;
    ros::Publisher batStatus_pub;

    ros::Publisher encoder_pub;
    ros::Publisher current_pub;
    ros::Publisher wheelPower_pub;
    ros::Publisher motorFeedbackDiffDrive_pub;

	ros::Publisher navSatFix_pub;
    tf::TransformBroadcaster br;

    
    double updateRate;

    double encoderSensorFreq;
    double wheelSensorFreq;
    double regulatorFreq;
    double setPowerFreq;
    double stateCheckFreq;
    double loopSensorFreq;
    double batteryCheckFreq;
    double GPSCheckFreq;
    double pitchRollFreq;
    double sensorStatusCheckFreq;

    ros::Duration timeSinceWheelSensor;
    ros::Duration timeSinceEncoderSensor;
    ros::Duration timeSinceRegulator;
    ros::Duration timeSinceState;
    ros::Duration timeSinceLoop;
    ros::Duration timeSincePitchRoll;
    ros::Duration timeSincebattery;
    ros::Duration timeSinceGPS;
    ros::Duration timeSinceStatus;
    ros::Duration timeSinceCollision;
    

    // Control data
    double lin_vel;
    double ang_vel;

    geometry_msgs::PoseStamped robot_pose;

    double xpos, ypos, yaw;
    double wanted_lv, wanted_rv;
    double wanted_power_left, wanted_power_right;
    double current_lv, current_rv;
    double last_yaw;
    int16_t power_l, power_r;

    ros::Time lastEncoderSampeTime;
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
    unsigned short requestedState;
    
    
    am_driver::WheelEncoder encoder;
    am_driver::WheelCurrent wheelCurrent;
    am_driver::WheelPower wheelPower;
    am_driver::MotorFeedbackDiffDrive motorFeedbackDiffDrive;

    int publishTf;
    int velocityRegulator;
    bool regulateBySpeed;

    bool newSound;
    char soundCmd[100];

    int collisionState;

    // Cutting disc
    bool cuttingDiscOn;
    bool lastCuttingDiscOn;
    unsigned char cuttingHeight;
    unsigned char lastCuttingHeight;

    uint8_t actionResponse;

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
    double lastRateCheckTime;
    int lastComtestWheelMotorPower;
    double startTime;

    bool startWithoutLoop;
    bool publishEuler;

    bool printCharge;
    am_driver::BatteryStatus batteryStatus;

    // UserStop
    bool userStop;
    
    // Accelerometer
    bool   m_PitchAndRollFromAccelerometer;
    double m_pitch;
    double m_roll;
    
  
    
	// GPS
	sensor_msgs::NavSatFix      m_navSatFix_msg;
	bool						m_publishGPS;

    tf::Transform transform;

};

typedef boost::shared_ptr<AutomowerSafe> AutomowerSafePtr;
}

#endif
