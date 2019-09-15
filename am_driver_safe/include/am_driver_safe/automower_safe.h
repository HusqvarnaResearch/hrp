/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *         Kent Askenmalm
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
#include <sensor_msgs/Joy.h>
#include <am_driver/WheelEncoder.h>
#include <am_driver/WheelCurrent.h>
#include <am_driver_safe/TifCmd.h>
#include <am_driver_safe/turnOfLoopCmd.h>
#include <am_driver/MotorFeedback.h>
#include <am_driver/MotorFeedbackDiffDrive.h>
#include <sensor_msgs/NavSatFix.h>
#include <am_driver/CurrentStatus.h>

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

#define AM_STATE_UNDEFINED     am_driver::SensorStatus::CONTROL_STATE_UNDEFINED
#define AM_STATE_IDLE          am_driver::SensorStatus::CONTROL_STATE_IDLE
#define AM_STATE_INIT          am_driver::SensorStatus::CONTROL_STATE_INIT
#define AM_STATE_MANUAL        am_driver::SensorStatus::CONTROL_STATE_MANUAL
#define AM_STATE_RANDOM        am_driver::SensorStatus::CONTROL_STATE_RANDOM
#define AM_STATE_PARK          am_driver::SensorStatus::CONTROL_STATE_PARK

// Serial port states
#define AM_SP_STATE_OFFLINE (0)
#define AM_SP_STATE_ONLINE (1)
#define AM_SP_STATE_CONNECTED (2)
#define AM_SP_STATE_INITIALISING (3)
#define AM_SP_STATE_ERROR (4)

// OPERATIONAL MODES (i.e. published in SensorStatus.operationalMode)
#define AM_OP_MODE_OFFLINE          (am_driver::SensorStatus::OPERATIONAL_MODE_OFFLINE)
#define AM_OP_MODE_CONNECTED_MANUAL (am_driver::SensorStatus::OPERATIONAL_MODE_MANUAL)
#define AM_OP_MODE_CONNECTED_RANDOM (am_driver::SensorStatus::OPERATIONAL_MODE_RANDOM)

// Sensor states
#define HVA_SS_HMB_CTRL     am_driver::SensorStatus::SENSOR_STATUS_HMB_CTRL
#define HVA_SS_OUTSIDE      am_driver::SensorStatus::SENSOR_STATUS_OUTSIDE
#define HVA_SS_COLLISION    am_driver::SensorStatus::SENSOR_STATUS_COLLISION
#define HVA_SS_LIFTED       am_driver::SensorStatus::SENSOR_STATUS_LIFTED
#define HVA_SS_TOO_STEEP    am_driver::SensorStatus::SENSOR_STATUS_TOO_STEEP
#define HVA_SS_PARKED       am_driver::SensorStatus::SENSOR_STATUS_PARKED
#define HVA_SS_IN_CS        am_driver::SensorStatus::SENSOR_STATUS_IN_CS
#define HVA_SS_USER_STOP    am_driver::SensorStatus::SENSOR_STATUS_USER_STOP
#define HVA_SS_CFG_NEEDED   am_driver::SensorStatus::SENSOR_STATUS_CFG_NEEDED
#define HVA_SS_DISC_ON      am_driver::SensorStatus::SENSOR_STATUS_DISC_ON
#define HVA_SS_LOOP_ON      am_driver::SensorStatus::SENSOR_STATUS_LOOP_ON
#define HVA_SS_CHARGING     am_driver::SensorStatus::SENSOR_STATUS_CHARGING

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




enum DriverTypes {
    SD_SAFE,
    SD_STRICT
};

class AutomowerSafe
{
public:
    AutomowerSafe(const ros::NodeHandle& nodeh, decision_making::RosEventQueue* eq);
    ~AutomowerSafe();

    bool setup();
    virtual bool update(ros::Duration dt);
    
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
    
    void clearOverRide();

    
    void newControlMainState(int aNewState);
    
    int GetUpdateRate();
    

protected:
    void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel);
    void powerCallback(const am_driver::WheelPower::ConstPtr& power);
    void modeCallback(const std_msgs::UInt16::ConstPtr& msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& j);

    void simLoopCallback(const am_driver::Loop& msg);
    bool firstLoopCallback = true;
    
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
    bool getLoopDetection();
    bool getStatus();
    bool getChargingPowerConnected();
    bool getStatusKeepAlive();
    bool getSensorStatus();
    bool getLoopData();
    bool getBatteryData();

    void setRandomMode();
    void setManualMode();

    bool isTimeOut(ros::Duration elapsedTime, double frequency);

    ssize_t sendMessage(unsigned char *msg, size_t len, unsigned char *ansmsg, size_t maxAnsLength, bool retry);

    bool turnOffLoop(am_driver_safe::turnOfLoopCmd::Request& req,
                                      am_driver_safe::turnOfLoopCmd::Response& res);

    bool executeTifCommand(am_driver_safe::TifCmd::Request& req,
                                      am_driver_safe::TifCmd::Response& res);

    double regulatePid(double current_vel, double wanted_vel);
    bool doSerialComTest();
    void handleCollisionInjections(ros::Duration dt);
    void sendGuideCommand(std::string cmd, double delaySec, ros::Duration dt);
    
    void handleFollowGuide(ros::Duration dt);


    std::string loadJsonModel(std::string fileName);

    // ROS data
    ros::NodeHandle nh;

    ros::ServiceServer tifCommandService;
    ros::ServiceServer turnOffLoopService;
    ros::Subscriber velocity_sub;
    ros::Subscriber power_sub;
    ros::Subscriber cmd_sub;
    ros::Subscriber joy_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber imu_euler_sub;
    ros::Subscriber sim_loop_sub;

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
    ros::Publisher currentStatus_pub;
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

    int followGuideState;
    ros::Duration guideTimer;

    // Parameters for wire following
    typedef enum 
    {
        wire_A_left = 0,
        wire_A_right,
        wire_G1,
        wire_G2,
        wire_G3
    } WireType;
    WireType wireToFollow;

    typedef enum 
    {
        CS_Range_min = 0,
        CS_Range_med = 350,
        CS_Range_max = 700,
    } CSRangeType;
    CSRangeType csRange;
    // Follow In
    int inCorridorMinWidth;
    int inCorridorMaxWidth;
    int wireInGuideCorridor;
    int autoDistanceEnabled;
    int delayTime;
    int followWireInEnable;

    //Follow out
    int startPositionId;
    int runningDistance;
    int proportion;
    int startPositionEnable;
    int minMaxDistance;

    // Parameters
    std::string pSerialPort;
    bool serialComTest;
    bool serialLog;

    // Nano controller
    bool joyDisabled;
    bool waitingForRelease;


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
    am_driver::CurrentStatus currentStatus;
    
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

    // Name of ROS TF frames
    std::string odomFrame;
    std::string baseLinkFrame;

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
    bool                        m_publishGPS;

    tf::Transform transform;

};

class AutomowerStrict : public AutomowerSafe
{
public:
    AutomowerStrict(const ros::NodeHandle& nodeh, decision_making::RosEventQueue* eq);
    bool update(ros::Duration dt);
private:
    bool getLoopA0Data();
    int updateCounter;
};

typedef boost::shared_ptr<AutomowerSafe> AutomowerSafePtr;

}

#endif
