/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Authors: Stefan Grufman
 * 			Kent Askenmalm
 *
 */

#include "am_driver_safe/automower_safe.h"
#include "am_driver_safe/amproto.h"
#include "am_driver/Mode.h"
#include <tf/transform_datatypes.h>

#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <fstream>
#include <sstream>

#define DEG2RAD(DEG) ((DEG) * ((M_PI) / (180.0)))

#define AM_POWER_INCDEC (1)
#define AM_POWER_THRESHOLD (0.1)

// Mower internal modes
#define IMOWERAPP_MODE_AUTO (0)
#define IMOWERAPP_MODE_MANUAL (1)
#define IMOWERAPP_MODE_HOME (2)
#define IMOWERAPP_MODE_DEMO (3)


// Mower internal states
#define IMOWERAPP_STATE_OFF              0
#define IMOWERAPP_STATE_WAIT_SAFETY_PIN  1
#define IMOWERAPP_STATE_STOPPED          2
#define IMOWERAPP_STATE_FATAL_ERROR      3
#define IMOWERAPP_STATE_PENDING_START    4
#define IMOWERAPP_STATE_PAUSED           5
#define IMOWERAPP_STATE_IN_OPERATION     6
#define IMOWERAPP_STATE_RESTRICTED       7
#define IMOWERAPP_STATE_ERROR            8


//#define DEBUG_LOG(X)  std::cout << X << std::endl;
#define DEBUG_LOG(X)

#define RADIANS_PER_DEGREE (3.14159/180.0)



namespace Husqvarna
{

#ifdef __cplusplus
extern "C"
{
#endif

//
// For HCP Runtime environment
//
static void* _malloc(hcp_Size_t size, void* ctx) {
    return malloc(size);
}

static void _free(void* dest, void* ctx) {
    free(dest);
}

static void* _memcpy(void* dest, const void* source, hcp_Size_t size, void*  ctx) {
    return memcpy(dest, source, size);
};

static void* _memset(void* dest, hcp_Int value, hcp_Size_t len, void*  ctx) {
    return memset(dest, value, len);
};

hcp_Int compareCodec(void* codec, void* name, void* pState) {
    tCodec* c = (tCodec*)codec;
    return hcp_szStrCmp(c->name, (const hcp_szStr)name) == 0;
}

hcp_Boolean isCodec(void* codec, void* pState) {
    tCodec* c = (tCodec*)codec;
    return c->lib != NULL ? HCP_TRUE : HCP_FALSE;
}

#ifdef __cplusplus
}
#endif




AutomowerSafe::AutomowerSafe(const ros::NodeHandle& nodeh, decision_making::RosEventQueue* eq)
{
    // Init attributes
    nh = nodeh;
    eventQueue= eq;
    lin_vel = 0.0;
    ang_vel = 0.0;

    wanted_lv = 0.0;
    wanted_rv = 0.0;

    current_lv = 0.0;
    current_rv = 0.0;

    // Parameters
    ros::NodeHandle n_private("~");
    std::string defPort = "/dev/ttyACM0";
    n_private.param("serialPort", pSerialPort, defPort);
    ROS_INFO("Param: serialPort: [%s]", pSerialPort.c_str());

    n_private.param("updateRate", updateRate, 1000.0);
    ROS_INFO("Param: updateRate: [%f]", updateRate);

    // Check if we shall publish the TF's?
    // It could be our "position" node that should do this...
    n_private.param("publishTf", publishTf, 1);
    ROS_INFO("Param: publishTf: [%d]", publishTf);

    n_private.param("velocityRegulator", velocityRegulator, 1);
    ROS_INFO("Param: velocityRegulator: [%d]", velocityRegulator);

    n_private.param("printCharge", printCharge, true);
    ROS_INFO("Param: printCharge: [%d]", printCharge);

    n_private.param("serialComTest", serialComTest, false);
    ROS_INFO("Param: serialComTest: [%d]", serialComTest);

    n_private.param("serialLog",serialLog , false);
    ROS_INFO("Param: serialLog: [%d]", serialLog);

    n_private.param("pitchAndRoll", m_PitchAndRollFromAccelerometer, false);
    ROS_INFO("Param: pitchAndRoll: [%d]", m_PitchAndRollFromAccelerometer);

    n_private.param("GPSCheckFreq", GPSCheckFreq, 0.0);
    ROS_INFO("Param: GPSCheckFreq: [%f]", GPSCheckFreq);

    n_private.param("sensorStatusCheckFreq", sensorStatusCheckFreq, 1.0);
    ROS_INFO("Param: sensorStatusCheckFreq: [%f]", sensorStatusCheckFreq);

    n_private.param("encoderSensorFreq", encoderSensorFreq, 10.0);
    ROS_INFO("Param: encoderSensorFreq: [%f]", encoderSensorFreq);

    n_private.param("batteryCheckFreq", batteryCheckFreq, 1.0);
    ROS_INFO("Param: batteryCheckFreq: [%f]", batteryCheckFreq);

    n_private.param("loopSensorFreq", loopSensorFreq, 1.0);
    ROS_INFO("Param: loopSensorFreq: [%f]", loopSensorFreq);

    n_private.param("wheelSensorFreq", wheelSensorFreq, 50.0);
    ROS_INFO("Param: wheelSensorFreq: [%f]", wheelSensorFreq);

    n_private.param("regulatorFreq", regulatorFreq, 50.0);
    ROS_INFO("Param: regulatorFreq: [%f]", regulatorFreq);

    n_private.param("setPowerFreq", setPowerFreq, 0.0);
    ROS_INFO("Param: setPowerFreq: [%f]", setPowerFreq);

    n_private.param("pitchRollFreq", pitchRollFreq, 0.0);
    ROS_INFO("Param: pitchRollFreq: [%f]", pitchRollFreq);

    n_private.param("stateCheckFreq", stateCheckFreq, 1.0);
    ROS_INFO("Param: stateCheckFreq: [%f]", stateCheckFreq);

    n_private.param("publishEuler", publishEuler, true);
    ROS_INFO("Param: publishEuler: [%d]", publishEuler);

    n_private.param("startWithoutLoop", startWithoutLoop, true);
    ROS_INFO("Param: startWithoutLoop: [%d]", startWithoutLoop);

    n_private.param<std::string>("odomFrame", odomFrame, "odom");
    ROS_INFO("Param: odomFrame: [%s]", baseLinkFrame.c_str());
    n_private.param<std::string>("baseLinkFrame", baseLinkFrame, "base_link");
    ROS_INFO("Param: baseLinkFrame: [%s]", baseLinkFrame.c_str());


    bool simulateLoop;
    n_private.param("simulateLoop", simulateLoop, false);
    ROS_INFO("Param: simulateLoop: [%d]", simulateLoop);
    // Setup some ROS stuff
    cmd_sub = nh.subscribe("cmd_mode", 5, &AutomowerSafe::modeCallback, this);

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 5);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 5);

    encoder_pub = nh.advertise<am_driver::WheelEncoder>("wheel_encoder", 5);
    current_pub = nh.advertise<am_driver::WheelCurrent>("wheel_current", 5);
    wheelPower_pub = nh.advertise<am_driver::WheelPower>("wheel_power", 5);

    motorFeedbackDiffDrive_pub = nh.advertise<am_driver::MotorFeedbackDiffDrive>("motor_feedback_diff_drive", 5);

    joyDisabled = true;
    waitingForRelease = false;

    // Guide Following
    followGuideState = 0;

    // Follow In - defaults
    inCorridorMinWidth = 0;
    inCorridorMaxWidth = 0;
    autoDistanceEnabled = 0;
    wireInGuideCorridor = 0;
    delayTime = 0;
    followWireInEnable = 1;

    //Follow out - defaults
    startPositionId = 1;
    runningDistance = 100;
    proportion = 100;
    startPositionEnable = 1;
    minMaxDistance = 1;

    velocity_sub = nh.subscribe("cmd_vel", 1, &AutomowerSafe::velocityCallback, this);
    power_sub = nh.subscribe("cmd_power", 1, &AutomowerSafe::powerCallback, this);
    joy_sub = nh.subscribe("nano2", 1, &AutomowerSafe::joyCallback, this);

    loop_pub = nh.advertise<am_driver::Loop>("loop", 5);
    sensorStatus_pub = nh.advertise<am_driver::SensorStatus>("sensor_status", 5);
     
    batStatus_pub  = nh.advertise<am_driver::BatteryStatus>("battery_status", 5, true);
    
    navSatFix_pub = nh.advertise<sensor_msgs::NavSatFix>("GPSfix", 5);

    currentStatus_pub = nh.advertise<am_driver::CurrentStatus>("current_status", 5);

    ROS_INFO("Param: publishEuler: [%d]", publishEuler);
    if (publishEuler)
    {
        euler_pub = nh.advertise<geometry_msgs::Vector3Stamped>("imu_euler", 1);
    }


    tifCommandService = nh.advertiseService("tif_command", &AutomowerSafe::executeTifCommand, this);
    ROS_INFO("Service /tif_command.");

    pidResetService = nh.advertiseService("reset_pid", &AutomowerSafe::resetPid, this);

    // Initialize the intial pose
    robot_pose.pose.position.x = 0.0;
    robot_pose.pose.position.y = 0.0;
    robot_pose.pose.position.z = 0.0;

    xpos = 0.0;
    ypos = 0.0;
    yaw = 0.0;

    lastLeftPulses = 0;
    lastRightPulses = 0;
    automowerInterfaceInited = false;
    requestedState = AM_STATE_MANUAL;

    tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
    robot_pose.pose.orientation.x = q.x();
    robot_pose.pose.orientation.y = q.y();
    robot_pose.pose.orientation.z = q.z();
    robot_pose.pose.orientation.w = q.w();

    robot_pose.header.frame_id = baseLinkFrame;
    robot_pose.header.stamp = ros::Time::now();


    cuttingDiscOn = false;
    lastCuttingDiscOn = false;

    cuttingHeight = 0;  // Not calibrated, undefined
    lastCuttingHeight = 0; // Not calibrated, undefined

    requestedLoopOn = true;
    newSound = false;

    collisionState = 0;

    serialPortState = AM_SP_STATE_OFFLINE;

    lastComtestWheelMotorPower = 3;

    actionResponse = 0;

    std::string tmp;
    //nh.param("jsonFile", tmp, (std::string) "./config/31.7_P2_Main_App-Certified_master_build_285-Debug.json");
    n_private.param("jsonFile", tmp, (std::string) "./config/31.7_Main-App-P2_master_build-542_Debug.json");
    jsonFile = tmp;

    timeSinceWheelSensor = ros::Duration(0.0);
    timeSinceEncoderSensor = ros::Duration(0.005);
    timeSinceRegulator = ros::Duration(0.010);
    timeSinceState = ros::Duration(0.015);
    timeSinceLoop = ros::Duration(0.025);
    timeSincePitchRoll = ros::Duration(0.035);
    timeSincebattery = ros::Duration(0.045);
    timeSinceGPS = ros::Duration(0.055);
    timeSinceStatus = ros::Duration(1.065);  // Make sure we read this from the beginning by setting 
    timeSinceCollision = ros::Duration(0.0);


    nextAutomowerInitTime = ros::Time::now();
    startTime = ros::Time::now().toSec();
    lastRateCheckTime = ros::Time::now().toSec();

    userStop = true; // Assume stopped...

    // Init the HCP library
    memset(&hcpHost, 0, sizeof(hcp_tHost));
    memset(&hcpCodecs, 0, sizeof(tCodecSet));

    hcpHost.free_ = _free;
    hcpHost.malloc_ = _malloc;
    hcpHost.memcpy_ = _memcpy;
    hcpHost.memset_ = _memset;

    // Initialize the internal state
    hcp_Int error;
    hcpState = (hcp_tState*)hcpHost.malloc_(hcp_SizeOfState(),hcpHost.context);
    if (hcp_NewState(hcpState, &hcpHost) != HCP_NOERROR)
    {
        ROS_ERROR("Could not initialize HCP State.");
    }
    else
    {
        error = HCP_INITIALIZEVECTOR(hcpState, &(hcpCodecs.header), hcpCodecs.fixed,
                                     tCodec, HCP_NULL, compareCodec, isCodec);

        if (error != HCP_NOERROR)
        {
            hcp_CloseState(hcpState);
            ROS_ERROR("Could not initialize HCP Vector.");
        }
    }

    std::string model = loadJsonModel(jsonFile);

    // Load the AMG3 codec
    hcp_tCodecLibrary* lib = hcp_GetLibrary();
    char codecName[5] = "amg3";
    error = hcp_LoadCodec(hcpState, lib, codecName, 5);

    if (error != HCP_NOERROR)
    {
        hcp_CloseState(hcpState);
        ROS_ERROR("Could not initialize Load AMG3 codec.");
    }

    error = hcp_LoadModel(hcpState, (hcp_cszStr)model.c_str(), sizeof(model.c_str()), &modelId);
    if (error != HCP_NOERROR)
    {
        hcp_CloseState(hcpState);
        ROS_ERROR("Could not Load JSON model.");
    }

    // Create a codec instance
    error = hcp_NewCodec(hcpState, codecName, (hcp_Size_t)modelId, &codecId);
    if (error != HCP_NOERROR)
    {
        hcp_CloseState(hcpState);
        ROS_ERROR("Could not create new Codec instance.");
    }

    m_regulatingActive = false;

    ROS_INFO("AutomowerSafe::Loaded HCP/TIF codec...let's go!");


    if(simulateLoop)
    {
        turnOffLoopService = nh.advertiseService("turnOfLoopDetection", &AutomowerSafe::turnOffLoop, this);
        ROS_INFO("Activating loop callback");
        sim_loop_sub = nh.subscribe("loop_sim", 1, &AutomowerSafe::simLoopCallback, this);

    }
}

AutomowerSafe::~AutomowerSafe()
{
    if (serialFd >= 0)
    {
        close(serialFd);
    }
}

bool AutomowerSafe::turnOffLoop(am_driver_safe::turnOfLoopCmd::Request& req,
                                am_driver_safe::turnOfLoopCmd::Response& res)
{
    eventQueue->raiseEvent("/LOOPDETECTION_CHANGED");
    return true;
}

ssize_t AutomowerSafe::sendMessage(unsigned char *msg, size_t len, unsigned char *ansmsg, size_t maxAnsLength, bool retry)
{
	
/*
 	std::cout << "SEND: " << std::hex;
	for (int i=0; i<len; i++)
	{
		std::cout << "0x" << (int)msg[i] << " ";
	}
	std::cout << std::dec << std::endl;
*/	
	
	// Sending
  ssize_t cnt=0;
	cnt = write(serialFd, msg, len);
	
	if(cnt != len)
	{
		ROS_ERROR("AutomowerHIL::Could not send on serial port!");
		return -1;
	}

	cnt = 0;
  ssize_t res;
  size_t payloadLength = 0;
	
	// Clear answer buffer
	memset(ansmsg, 0, maxAnsLength);
	
	// Keep reading until we find an STX
	
	// receive STX
	while (ansmsg[cnt] != 0x02)
	{
		res = read(serialFd, &ansmsg[cnt], 1);
	}
	cnt++;

	// Read MESSAGE TYPE
	res = read(serialFd, &ansmsg[cnt], 1);
	//std::cout << "MESSAGE TYPE: " << (int)ansmsg[cnt] << std::endl;
	cnt++;
	
	// Read LENGTH
	res = read(serialFd, &ansmsg[cnt], 1);
	payloadLength = ansmsg[cnt];
	//std::cout << "PAYLOAD LENGTH: " << (int)payloadLength << std::endl;
	cnt++;
	
	// Read PAYLOAD
	unsigned char readBytes = 0;
	unsigned max_retries = 100;
	while ((readBytes < payloadLength) && (max_retries > 0))
	{
    ssize_t bytes = read(serialFd, &ansmsg[cnt], payloadLength-readBytes);
		if (bytes > 0)
		{
			readBytes += bytes;
			cnt += bytes;
		}
		max_retries--;
	}
	
	// Read CRC
	// TODO: Use it!
	res = read(serialFd, &ansmsg[cnt], 1);
	cnt++;

	// Read ETX
	res = read(serialFd, &ansmsg[cnt], 1);
	cnt++;

/*
	std::cout << "CNT: " << cnt << std::endl;
	std::cout << "RESPONSE: " << std::hex;
	for (int i=0; i<cnt; i++)
	{
		std::cout << "0x" << (int)ansmsg[i] << " ";
	}
	std::cout << std::dec << std::endl;
*/

	// Check some stuff
	if (ansmsg[0] != 0x02)
	{
		// FAILED
		return 0;
	}
	if (ansmsg[cnt-1] != 0x03)
	{
		// FAILED
		return 0;
	}

	return cnt;

}

void AutomowerSafe::simLoopCallback(const am_driver::Loop& msg)
{
    ROS_INFO("Recieving Loop values");
    hcp_tResult result;
    if(firstLoopCallback)
    {
        ROS_INFO("First run!");
        firstLoopCallback = false;
        const char* msg1 = "GardenSimulator.SetSimulator(id:7,value:100)";

        if (!sendMessage(msg1, sizeof(msg1), result))
        {
            ROS_WARN("Error in setting loop quality!");
        }
        return;
    }

    char msg1[100];

    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:8,value:%i)",msg.A0.frontCenter);
    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:9,value:%i)",msg.A0.frontRight);
    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:10,value:%i)",msg.A0.rearLeft);

    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:11,value:%i)",msg.A0.rearRight);

    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
/*
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:12,value:%i)",msg.G1.frontCenter);
    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:13,value:%i)",msg.G1.frontRight);
    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:14,value:%i)",msg.G1.rearLeft);

    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:15,value:%i)",msg.G1.rearRight);

    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:16,value:%i)",msg.G2.frontCenter);
    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:17,value:%i)",msg.G2.frontRight);
    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:18,value:%i)",msg.G2.rearLeft);
    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:19,value:%i)",msg.G2.rearRight);

    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:20,value:%i)",msg.G3.frontCenter);
    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:21,value:%i)",msg.G3.frontRight);
    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:22,value:%i)",msg.G3.rearLeft);
    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:23,value:%i)",msg.G3.rearRight);

    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:24,value:%i)",msg.N.frontCenter);
    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:25,value:%i)",msg.N.frontRight);
    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:26,value:%i)",msg.N.rearLeft);
    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:27,value:%i)",msg.N.rearRight);

    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    */
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:28,value:%i)",msg.F.frontCenter);
    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:29,value:%i)",msg.F.frontRight);
    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:30,value:%i)",msg.F.rearLeft);
    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }
    snprintf(msg1, sizeof(msg1), "GardenSimulator.SetSimulator(id:31,value:%i)",msg.F.rearRight);

    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_WARN("Error in sending loop-values!");
    }

}

bool AutomowerSafe::executeTifCommand(am_driver_safe::TifCmd::Request& req,
                                      am_driver_safe::TifCmd::Response& res)
{
    hcp_tResult result;
    const char* msg = req.str.c_str();
    ROS_INFO("executeTifCommand %s...", msg);
    if (!sendMessage(msg, sizeof(msg), result))
    {
        return false;
    }
    ROS_INFO("... result.message: %s", result.message);
    std::string strRes;
    strRes = resultToString(result);
    std::cout << strRes << std::endl;
    res.str =  strRes;

    return true;
}

bool AutomowerSafe::resetPid(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
{
  leftWheelPid.Restart();
  rightWheelPid.Restart();
  return true;
}

std::string AutomowerSafe::loadJsonModel(std::string fileName)
{
    std::string line;
    std::ifstream file(fileName.c_str());
    std::cout << "Loading from:" << fileName << "..." << std::endl;

    std::string str;
    std::string fileContents;
    if (file.is_open())
    {
        while (std::getline(file, str))
        {
            fileContents += str;
            fileContents.push_back('\n');
        }
        return fileContents;
    }
    else
    {
        ROS_ERROR("Could not load JSON file!!!");
        return str;
    }
}


bool AutomowerSafe::setup()
{
    struct termios term;

    // Open serial port

    serialFd = open(pSerialPort.c_str(), O_RDWR /* | O_NONBLOCK */);
    close(serialFd);
    serialFd = open(pSerialPort.c_str(), O_RDWR /* | O_NONBLOCK */);
    //    serial_struct serinfo;
    //    //Enable low latency communication
    //    ioctl(serialFd, TIOCGSERIAL, &serinfo);
    //    serinfo.flags |= ASYNC_LOW_LATENCY;
    //    ioctl(serialFd, TIOCSSERIAL, &serinfo);
    if (serialFd < 0)
    {
        if (errno == ENOENT)
        {
            ROS_ERROR("Automower::Serial port don't exist ");
        }
        else if (errno == EBUSY)
        {
            ROS_ERROR("Automower::Serial port already open ");
        }
        else
        {
            ROS_ERROR("Automower::Serial port open failed. Errorcode %d ", errno);
        }

        return false;
    }

    memset(&term, 0, sizeof(term));

    /* man termios get more info on below settings */
    /* CS8 - character size mask
     * CLOCAL - ignore modem control lines
     * CREAD - enable receiver
     */
    term.c_cflag = B115200 | CS8 | CLOCAL | CREAD; /* control modes */
    term.c_iflag = 0;                              /* input mode */
    term.c_oflag = 0;                              /* output modes */
    term.c_lflag = 0;                              /* local modes */

    tcflush(serialFd, TCIFLUSH);
    term.c_cflag |= (CLOCAL | CREAD);

    /*
    VMIN specifies the minimum number of bytes to read before read() returns.
    VTIME is the time to wait (in tenths of a second) before returning from
    read (). VMIN and VTIME are used only when the ICANON flag is clear in
    the c_lflag parameter. In the previous example, you probably noticed
    that c_cc[VMIN] and c_cc[VTIME] were set to 1 to enable non-blocking
    reads. Many programmers are confused by VMIN and VTIME, but the rules are
    reasonably simple; four possible combinations of VMIN and VTIME are
    possible, as follows (from POSIX.1, 7.1.1.7):

    VMIN > 0 and VTIME > 0
            TIME is an interbyte timer that is activated after the first byte
            is received and is reset after each received byte. If the timer
            expires, at least one byte has been received because the timer
            is not activated until after a byte has been received.
    VMIN > 0 and VTIME = 0
            There is no timer, so read() returns when MIN bytes have been
            received or when a signal is received.
    VMIN = 0 and VTIME > 0
            Because MIN is 0, TIME is a read () operation timer that is
            activated as soon as read() is called. The read() operation
            returns as soon as a byte is received or when the timer expires.
    VMIN = 0 and VTIME = 0
            The read() operation returns immediately with the data that is
            in the input buffer. If the input buffer is empty, read()
            returns immediately with zero bytes of data.
    */

    cfsetospeed(&term, (speed_t)B115200);
    cfsetispeed(&term, (speed_t)B115200);

    term.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    term.c_cflag &= ~CSIZE;
    term.c_cflag |= CS8;         /* 8-bit characters */
    term.c_cflag &= ~PARENB;     /* no parity bit */
    term.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    term.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    term.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    term.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    term.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    term.c_cc[VMIN] = 1;
    term.c_cc[VTIME] = 0;
    tcsetattr(serialFd, TCSANOW, &term);

    ROS_INFO("Serial setup complete");

    return true;
}
void AutomowerSafe::imuResetCallback(const geometry_msgs::Pose::ConstPtr& msg)
{

    ROS_INFO("Automower::imuResetCallback!");
    // Set heading from orientation
    tf::Quaternion q;
    double r, p, y;
    tf::quaternionMsgToTF(msg->orientation, q);
    tf::Matrix3x3(q).getRPY(r, p, y);
    yaw = y;

    // Set pose to the IMU Positions
    xpos = msg->position.x;
    ypos = msg->position.y;
    //~ zpos = msg->position.z;
}

void AutomowerSafe::velocityCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
    velocityRegulator = true;

    lin_vel = (double)vel->linear.x;
    ang_vel = (double)vel->angular.z;

    wanted_lv = lin_vel - ang_vel * AUTMOWER_WHEEL_BASE_WIDTH / 2;
    wanted_rv = lin_vel + ang_vel * AUTMOWER_WHEEL_BASE_WIDTH / 2;

    // ROS_INFO("Automower::cmd_vel: %f m/s  %f rad/s => wanted_lv=%f, wanted_rv=%f", (float)lin_vel, (float)ang_vel,
    // (float)wanted_lv, (float)wanted_rv);
}

void AutomowerSafe::powerCallback(const am_driver::WheelPower::ConstPtr& power)
{
    velocityRegulator = false;

    wanted_power_left = power->left;
    wanted_power_right = power->right;

    ROS_INFO("wanted_power: %f", wanted_power_left);
}

void AutomowerSafe::joyCallback(const sensor_msgs::Joy::ConstPtr& j)
{

    double dummy;
    double sliderScale[8] = {50, 50, 50, 50, 50, 50, 50, 50};
    double sliderOffset[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    double *sliderVar[8] = {&wheelSensorFreq, &encoderSensorFreq,&regulatorFreq,&loopSensorFreq,&dummy,&dummy,&dummy,&dummy};
    double knobScale[8] = {50, 50, 50, 50, 50, 50, 50, 50};
    double knobOffset[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    double *knobVar[8] = {&dummy, &dummy,&dummy,&dummy,&dummy,&dummy,&dummy,&dummy};

    // Special case to handle key release event...
    if (waitingForRelease)
    {
        waitingForRelease = false;
        return;
    }

    // Need to "enable" this before use of params...
    if (joyDisabled)
    {
        // Pressing R[7]
        if (j->buttons[25 + 16 + 7] == 1)
        {
            joyDisabled = false;
            waitingForRelease = true;
            ROS_INFO("FusePos::NANO KONTROL enabled!");
        }

        // Always return...
        return;
    }
    else
    {
        // Pressing  R[0]
        if (j->buttons[25 + 16 + 0] == 1)
        {
            joyDisabled = true;
            waitingForRelease = true;
            ROS_INFO("FusePos::NANO KONTROL disabled!");
            return;
        }
        // REC
        else if (j->buttons[8] == 1)
        {

            waitingForRelease = true;
            ROS_INFO("FusePos::NANO KONTROL start recording!");
            return;
        }
    }

    if (j->buttons[1] == 1)
    {
        waitingForRelease = true;
    }
    else
    {
        ROS_INFO("#### NEW PARAMETERS ####");

        // Live tweaking of params...
        //
        // SLIDERS
        //
        for (int i = 0; i < 8; i++)
        {
            if (j->axes[i] != 0)
            {
                *sliderVar[i] = sliderScale[i] * j->axes[i] + sliderOffset[i];
                ROS_INFO("AutomowerSafe::sliderVar[%d] = %f",i, *sliderVar[i]);
            }
        }

        //
        // KNOBS
        //
        for (int i = 0; i < 8; i++)
        {
            if (j->axes[8 + i] != 0)
            {
                *knobVar[i] = knobScale[i] * j->axes[8 + i] + knobOffset[i];
                ROS_INFO("AutomowerSafe::knobVar[%d] = %f",i, *knobVar[i]);
            }
        }
    }
}

void AutomowerSafe::setManualMode()
{
        requestedState = AM_STATE_MANUAL;
        ROS_INFO("AutoMowerSafe: Manual Mode Requested");
        eventQueue->raiseEvent("/MANUAL");
}

void AutomowerSafe::setRandomMode()
{
        requestedState = AM_STATE_RANDOM;
        ROS_INFO("AutoMowerSafe: Random Mode Requested");
        eventQueue->raiseEvent("/RANDOM");
}

void AutomowerSafe::modeCallback(const std_msgs::UInt16::ConstPtr& msg)
{
    using am_driver::Mode;
    if (msg->data < Mode::MODE_MANUAL)
    {
        // Not for us...
        return;
    }

    if (msg->data == Mode::MODE_MANUAL)
    {
        setManualMode();
    }
    else if (msg->data == Mode::MODE_RANDOM)
    {
        setRandomMode();
    }
    else if (msg->data == Mode::MODE_CUTTING_DISC_OFF)
    {
        cuttingDiscOn = false;
        eventQueue->raiseEvent("/CUTDISC_CHANGED");
        ROS_INFO("AutoMowerSafe: Cutting Disc OFF");
    }
    else if (msg->data == Mode::MODE_CUTTING_DISC_ON)
    {
        cuttingDiscOn = true;
        eventQueue->raiseEvent("/CUTDISC_CHANGED");
        ROS_INFO("AutoMowerSafe: Cutting Disc ON");
    }
    else if (msg->data == Mode::MODE_CUTTING_HEIGHT_60)
    {
        cuttingHeight = 60;
        eventQueue->raiseEvent("/CUTTINGHEIGHT_CHANGED");
        ROS_INFO("AutoMowerSafe: Cutting Height = 60mm");

    }
    else if (msg->data == Mode::MODE_CUTTING_HEIGHT_40)
    {
        cuttingHeight = 40;
        eventQueue->raiseEvent("/CUTTINGHEIGHT_CHANGED");
        ROS_INFO("AutoMowerSafe: Cutting Height = 40mm");

    }
    else if (msg->data == Mode::MODE_PARK)
    {
        requestedState = AM_STATE_PARK;
        DEBUG_LOG(" raiseEvent PARKING");
        eventQueue->raiseEvent("/PARKING");
        ROS_INFO("AutoMowerSafe: Parking requested");
    }
    else if (msg->data == Mode::MODE_LOOP_ON)
    {
        requestedLoopOn = true;
        eventQueue->raiseEvent("/LOOPDETECTION_CHANGED");
        ROS_INFO("AutoMowerSafe: Loop detection on");
    }
    else if (msg->data == Mode::MODE_LOOP_OFF)
    {
        requestedLoopOn = false;
        eventQueue->raiseEvent("/LOOPDETECTION_CHANGED");
        ROS_INFO("AutoMowerSafe: Loop detection off");
    }
    else if (msg->data == Mode::MODE_COLLISION_INJECT)
    {
        ROS_INFO("AutoMowerSafe: Collision injected!");
        collisionState = 1;
    }
    else if (msg->data == Mode::MODE_COLLISIONS_DISABLED)
    {
        ROS_INFO("AutoMowerSafe: Collisions Disabled!");
        collisionState = 5;
    }
    else if (msg->data == Mode::MODE_COLLISIONS_ENABLED)
    {
        ROS_INFO("AutoMowerSafe: Collisions Enabled!");
        collisionState = 7;
    }

    else if (msg->data == Mode::MODE_SHUTDOWN)
    {
        ROS_WARN("Shutdown...bye bye...");
        if (!system("sudo shutdown -h now"))
        {
          ROS_ERROR("Failed to shut down");
        }
    }
    else if (msg->data == Mode::MODE_REBOOT)
    {
        ROS_WARN("Rebooting...bye bye...");
        if (!system("sudo reboot"))
        {
          ROS_ERROR("Failed to reboot");
        }
    }
    // Sound commands, 0x400 is a nice little beep and then they get increasingly annoying (full Alarm 10 minutes for example)
    // Use 0x40E to shut the sound off
    else if (msg->data >= Mode::MODE_SOUND_KEY_CLICK && msg->data <= Mode::MODE_SOUND_OFF )
    {
        newSound = true;
        int soundType = msg->data - Mode::MODE_SOUND_KEY_CLICK;

        snprintf(soundCmd, sizeof(soundCmd), "Sound.SetSoundType(soundType:%d) ",soundType);
    }
    // 0x410: Stop folloing guides and enter MANUAL mode (need to actively switch to RANDOM if desired)
    else if (msg->data == Mode::MODE_FOLLOW_MANUAL)
    {
        followGuideState = 255;
    }
    //0x411: Follow G1, this may be laid out as an Island if an infinite follow loop sequence is desired
    else if (msg->data == Mode::MODE_FOLLOW_G1)
    {
        followGuideState = 1;
        wireToFollow = wire_G1;
    }
    //0x412: Follow G2, this can be used to actually get home after following the infinite loop G1, if laid out properly.
    else if (msg->data == Mode::MODE_FOLLOW_G2)
    {
        followGuideState = 1;
        wireToFollow = wire_G2;
    }
    //0x413: Follow G3, may be used on 450x (where there actually is a third guide, not on 430X)
    else if (msg->data == Mode::MODE_FOLLOW_G3)
    {
        followGuideState = 1;
        wireToFollow = wire_G3;
    }
    else
    {
        // Do nothing...probably not for me...
        return;
    }
}

bool AutomowerSafe::sendMessage(const char* msg, int len, hcp_tResult& result)
{

    // std::cout << msg << std::endl;
    static boost::mutex mtx_serial;

    mtx_serial.lock();

    double endSendTime;

    if ((serialPortState == AM_SP_STATE_OFFLINE) || (serialPortState == AM_SP_STATE_ERROR))
    {
        mtx_serial.unlock();
        ROS_ERROR("sendMessage: serialPort is OFFLINE or in ERROR mode - %s", msg);
        return false;
    }

    hcp_Uint8 buf[255];
    hcp_Int numBytes = 0;

    numBytes = hcp_Encode(hcpState, codecId, (hcp_cszStr)msg, buf, 255);

    if (numBytes <0)
    {
        if (numBytes == HCP_COMMANDNOTLOADED)
        {
            std::cout <<  msg << std::endl;
            ROS_ERROR("JSON file does not support command:  %s ", msg);
        }
        else
        {
            ROS_ERROR("JSON encoding failed with error %d for command %s ", numBytes, msg);
        }

        serialPortState = AM_SP_STATE_ERROR;
        return false;
    }

    if (serialLog) {
        std::cout << "SEND: " << std::hex;
        for (int i=0; i<numBytes; i++)
        {
            std::cout << std::hex << (int)buf[i] << " ";
        }
        std::cout << std::dec;
    }

    ssize_t cnt = 0;
    cnt = write(serialFd, buf, numBytes);

    if (cnt != numBytes)
    {
        ROS_ERROR("Automower::Could not send on serial port - %s", msg);
        serialPortState = AM_SP_STATE_ERROR;
        return false;
    }
    endSendTime = ros::Time::now().toSec()-startTime;

    numBytes = 0; // to be assigned after read again
    cnt = 0;
    ssize_t res;

    // Clear answer buffer (only using one byte...but reused from above)
    memset(buf, 0, 255);

    memset(&result, 0, sizeof(result));

    // Read one byte
    res = read(serialFd, &buf[0], 1);

    //    double responseTime = ros::Time::now().toSec() - startTime - endSendTime;
    double responseTime = 0;



    if (res > 0 && buf[0] > 0)
    {
        if (serialLog) { std::cout << endSendTime << "\t" << responseTime << "\t READ:  " << std::hex << (int)buf[0] << " " << std::flush; }

        cnt++;
        numBytes = hcp_Decode(hcpState, codecId, buf, 1, &result);
    }
    else
    {
        if ((res == 1) && (buf[0]==0))
        {
            ROS_WARN("Automower::Error receiving...rebooting??");
        }
        else
        {
            ROS_WARN("Automower::Failed to get response...sleeping?");
        }

        if (serialLog) {std::cout << "\n first read byte is zero => FAILED!  res = " << res << "  buf[0]= " << int(buf[0]) << " errno =" << errno  << "respTime " << responseTime  <<std::endl;}
        serialPortState = AM_SP_STATE_ERROR;
        mtx_serial.unlock();
        ROS_ERROR("sendMessage: Failed while trying - %s", msg);
        return false;
    }

    // TODO: Give up?
    while (((res > 0) && (result.command.length == 0)) || (result.error != HCP_NOERROR))
    {
        res = read(serialFd, &buf[0], 1);

        if (res > 0)
        {
            cnt++;
            numBytes = hcp_Decode(hcpState, codecId, buf, 1, &result);
            if (serialLog) { std::cout << std::hex << (int)buf[0] << " " << std::flush; }
        }
        else
        {
            if ((res == 1) && (buf[0]==0))
            {
                ROS_WARN("Automower::Error receiving ...... Automower rebooting??");
            }
            else
            {
                ROS_WARN("Automower::Failed to get response......sleeping?");
            }

            if (serialLog) {std::cout << "first read byte is zero => FAILED!  res = " << res << "  buf[0]= " << int(buf[0]) << " errno =" << errno  << "respTime " << responseTime  <<std::endl;}
            serialPortState = AM_SP_STATE_ERROR;
            ROS_ERROR("sendMessage: Failed while trying - %s", msg);
            return false;
        }
    }

    if (serialLog) {  std::cout << std::dec << " " << msg << " " << std::endl; }

    if (res <= 0)
    {
        ROS_WARN("Automower::Failed to get response...sleeping?");
        serialPortState = AM_SP_STATE_ERROR;

        mtx_serial.unlock();
        ROS_ERROR("sendMessage: Failed while trying - %s", msg);
        return false;
    }

    if (result.error != HCP_NOERROR)
    {
        ROS_WARN("Automower::Error receiving...not logged in?");
        mtx_serial.unlock();
        ROS_ERROR("sendMessage: Failed while trying - %s", msg);
        return false;
    }



    //std::cout << "SAFE::result.parameterCount= " << result.parameterCount << std::endl;

    mtx_serial.unlock();
    return true;
}

bool AutomowerSafe::initAutomowerBoard()
{
    ROS_INFO("Automower::initAutomowerBoard");


    hcp_tResult result;
    const char* msg = "DeviceInformation.GetDeviceIdentification()";
    if (!sendMessage(msg, sizeof(msg), result))
    {
        ROS_WARN("Failed to initAutomowerBoard");
        return false;
    }

    uint8_t deviceType = result.parameters[0].value.u8;
    std::cout << "DeviceType: " << (int)deviceType << std::endl;

    uint8_t mowerType = result.parameters[1].value.u8;
    std::cout << "MowerType: " << (int)mowerType << std::endl;

    uint32_t mowerSerial = result.parameters[2].value.u32;
    std::cout << "SerialNo: " << (int)mowerSerial << std::endl;

    uint8_t mowerVariant = result.parameters[3].value.u8;
    std::cout << "MowerVariant: " << (int)mowerVariant << std::endl;

    if (deviceType != 10)
    {
        ROS_ERROR("MowerDevice not found...");
        return false;
    }

    if ((mowerType == 5) || (mowerType == 7) || (mowerType == 8) || (mowerType == 16))    // 420, 430X, 450X OR 550
    {
        // Get some stuff out from the mower...
        AUTMOWER_WHEEL_BASE_WIDTH = 0.464500;
        WHEEL_DIAMETER = 0.245;
        WHEEL_PULSES_PER_TURN = 349;
        WHEEL_METER_PER_TICK = (2.0 * M_PI * WHEEL_DIAMETER / 2.0) / (double)WHEEL_PULSES_PER_TURN;
    }
    else if (mowerType == 14)   // P0
    {
        AUTMOWER_WHEEL_BASE_WIDTH = 0.3314; // Jonathan Björn
        WHEEL_DIAMETER = 0.210;
        WHEEL_PULSES_PER_TURN = 1192;
        WHEEL_METER_PER_TICK = (2.0 * M_PI * WHEEL_DIAMETER / 2.0) / (double)WHEEL_PULSES_PER_TURN;
    }
    else if (mowerType == 10 || mowerType == 4) //P1
    {
        //Best guesses... todo check really
        AUTMOWER_WHEEL_BASE_WIDTH = 0.3314; // Jonathan Björn
        WHEEL_DIAMETER = 0.210;
        WHEEL_PULSES_PER_TURN = 1188; //12*1:99 according to floor one
        WHEEL_METER_PER_TICK = (2.0 * M_PI * WHEEL_DIAMETER / 2.0) / (double)WHEEL_PULSES_PER_TURN;
    }
    else if (mowerType == 13) //P15 Gardena
    {
        //Best guesses... todo check really
        AUTMOWER_WHEEL_BASE_WIDTH = 0.370; // Jonathan Björn
        WHEEL_DIAMETER = 0.210;
        WHEEL_PULSES_PER_TURN = 1192; 
        WHEEL_METER_PER_TICK = (2.0 * M_PI * WHEEL_DIAMETER / 2.0) / (double)WHEEL_PULSES_PER_TURN;
    }

    ROS_INFO("Automower::WHEEL_DIAMETER = %f", WHEEL_DIAMETER);
    ROS_INFO("Automower::AUTMOWER_WHEEL_BASE_WIDTH = %f", AUTMOWER_WHEEL_BASE_WIDTH);
    ROS_INFO("Automower::WHEEL_PULSES_PER_TURN = %d", WHEEL_PULSES_PER_TURN);
    ROS_INFO("Automower::WHEEL_METER_PER_TICK = %f", WHEEL_METER_PER_TICK);


    // PID parameters set for 50Hz
    // x factor Adjust to equal regulation as for 50 Hz


    double x = 50.0 / regulatorFreq;

    leftWheelPid.Init(50.0, 10.0*x, 1.0/x);
    rightWheelPid.Init(50.0, 10.0*x, 1.0/x);

    lastComtestWheelMotorPower = 15;

    char msg1[100];
    snprintf(msg1, sizeof(msg1), "MowerApp.SetMode(modeOfOperation:%d)",IMOWERAPP_MODE_AUTO);

    if (!sendMessage(msg1, sizeof(msg1), result))
    {
        ROS_ERROR("Automower::Failed setting Auto Mode.");
        cuttingDiscOn = lastCuttingDiscOn;
        return false;
    }

    if (startWithoutLoop)
    {
        requestedLoopOn = false;
        eventQueue->raiseEvent("/LOOPDETECTION_CHANGED");
        ROS_INFO("AutoMowerSafe: Loop detection off");
    }

    pauseMower();

    return true;
}

bool AutomowerSafe::getEncoderData()
{
    ros::Time current_time = ros::Time::now();
    hcp_tResult result;

    //
    // Get the Rotation Counter
    //
    const char* leftCounterMsg = "Wheels.GetRotationCounter(index:1)";
    if (!sendMessage(leftCounterMsg, sizeof(leftCounterMsg), result))
    {
        return false;

    }
    if (result.parameterCount > 0)
    {
        leftPulses = -result.parameters[0].value.i32;
        motorFeedbackDiffDrive.left.header.stamp = ros::Time::now();
        motorFeedbackDiffDrive.left.ticks = leftPulses;
    }


    const char* rightCounterMsg = "Wheels.GetRotationCounter(index:0)";
    if (!sendMessage(rightCounterMsg, sizeof(rightCounterMsg), result))
    {
        return false;
    }
    if (result.parameterCount > 0)
    {
        rightPulses = -result.parameters[0].value.i32;
        motorFeedbackDiffDrive.right.header.stamp = ros::Time::now();
        motorFeedbackDiffDrive.right.ticks = rightPulses;
    }



    // Set the timestamp of the received data to now
    encoder.header.stamp = current_time;
    return true;
}

bool AutomowerSafe::getWheelData()
{ 
    ros::Time current_time = ros::Time::now();
    hcp_tResult result;

    const char* msg = "RealTimeData.GetWheelMotorData()";
    if (!sendMessage(msg, sizeof(msg), result))
    {
        return false;
    }

    motorFeedbackDiffDrive.header.stamp = current_time;

    int16_t tmpValue;
    tmpValue = result.parameters[1].value.i16;
    current_lv = ((double)tmpValue) / 1000.0;
    wheelPower.left = std::copysign(result.parameters[0].value.i16, power_l); // Not entierly correct. Uses the sign of the sent power. But may not be the same as the ongoing.
    wheelCurrent.left = result.parameters[2].value.i16;

    tmpValue = result.parameters[4].value.i16;
    current_rv = ((double)tmpValue) / 1000.0;
    wheelPower.right = std::copysign(result.parameters[3].value.i16, power_r); // Not entierly correct. Uses the sign of the sent power. But may not be the same as the ongoing.
    wheelCurrent.right = result.parameters[5].value.i16;

    wheelCurrent.header.stamp = current_time;
    wheelCurrent.header.frame_id = odomFrame;

    wheelPower.header.stamp = current_time;
    wheelPower.header.frame_id = odomFrame;


    motorFeedbackDiffDrive.left.omega = current_lv / (0.5 * WHEEL_DIAMETER);
    motorFeedbackDiffDrive.left.current = ((double)result.parameters[2].value.i16 / 1000);
    // Not entierly correct. Uses the sign of the sent power. But may not be the same as the ongoing.
    motorFeedbackDiffDrive.left.controlPower = std::copysign(result.parameters[0].value.i16, power_l);
    motorFeedbackDiffDrive.left.controlOmega = wanted_lv / (0.5 * WHEEL_DIAMETER);


    motorFeedbackDiffDrive.right.omega = current_rv / (0.5 * WHEEL_DIAMETER);
    motorFeedbackDiffDrive.right.current = ((double)result.parameters[5].value.i16 / 1000);
    // Not entierly correct. Uses the sign of the sent power. But may not be the same as the ongoing.
    motorFeedbackDiffDrive.right.controlPower = std::copysign(result.parameters[3].value.i16, power_r);
    motorFeedbackDiffDrive.right.controlOmega = wanted_rv / (0.5 * WHEEL_DIAMETER);

    return true;
}

std::string AutomowerSafe::resultToString(hcp_tResult result)
{
    int i;
    std::ostringstream resultTmp;

    std::string resultStr = "";
    std::string typeName;
    double value = 0.0;
    if (result.parameterCount < 0 || result.parameterCount > 100)
    {
        std::cout << "invalid number of parameters: " << result.parameterCount << std::endl;
        return "error";
    }
    for (i = 0; i  < result.parameterCount; i++)
    {

        std::string valueStr = "";
        switch (result.parameters[i].template_->type)
        {

        case HCP_FLOAT_ID:
            typeName = HCP_FLOAT_NAME;
            value = result.parameters[i].value.f;
            break;
        case HCP_BOOLEAN_ID:
            typeName = HCP_BOOLEAN_NAME;
            value = result.parameters[i].value.b;
            break;
        case HCP_VOID_ID:
            typeName = HCP_VOID_NAME;
            //value = result.parameters[i].value.p;
            std::cout << "no supported conversion for " << typeName << std::endl;
            break;
        case HCP_SIZET_ID:
            typeName = HCP_SIZET_NAME;
            value = result.parameters[i].value.sz;
            break;
        case HCP_UINT8_ID:
            typeName =  HCP_UINT8_NAME;
            value = result.parameters[i].value.u8;
            break;
        case HCP_INT8_ID:
            typeName = HCP_INT8_NAME;
            value = result.parameters[i].value.s8;
            break;
        case HCP_UINT16_ID:
            value = result.parameters[i].value.u16;
            typeName = HCP_UINT16_NAME;
            break;
        case HCP_INT16_ID:
            value = result.parameters[i].value.i16;
            typeName = HCP_INT16_NAME;
            break;
        case HCP_UINT32_ID:
            value = result.parameters[i].value.u32;
            typeName = HCP_UINT32_NAME;
            break;
        case HCP_INT32_ID:
            value = result.parameters[i].value.i32;
            typeName = HCP_INT32_NAME;
            break;
        case HCP_UINT64_ID:
            value = result.parameters[i].value.u64;
            typeName = HCP_UINT64_NAME;
            break;
        case HCP_INT64_ID:
            value = result.parameters[i].value.i64;
            typeName = HCP_INT64_NAME;
            break;
        case HCP_STRING_ID:
            valueStr = result.parameters[i].value.str.value;
            typeName = HCP_STRING_NAME;
            break;
        case HCP_DOUBLE_ID:
            value = result.parameters[i].value.d;
            typeName = HCP_DOUBLE_NAME;
            break;
        case HCP_BLOB_ID:
            //value = result.parameters[i].value.f;
            valueStr = "no supported conversion";
            std::cout << "no supported conversion for " << typeName << std::endl;
            typeName = HCP_BLOB_NAME;
            break;
        case HCP_UNIXTIME_ID:
            //value = result.parameters[i].value.time;
            valueStr = "no supported conversion";
            std::cout << "no supported conversion for " << typeName << std::endl;
            typeName = HCP_UNIXTIME_NAME;
            break;
        case HCP_SIMPLEVERSION_ID:
            typeName = HCP_SIMPLEVERSION_NAME;
            valueStr = "no supported conversion";
            std::cout << "no supported conversion for " << typeName << std::endl;
            break;
        case HCP_INVALID:
        default:
            typeName = HCP_INVALID_NAME;
            break;
        }
        if (valueStr.length() > 0)
        {
            resultTmp <<  i << "|" << typeName << "|" << valueStr << std::endl;
        }
        else
        {
            resultTmp <<  i << "|" << typeName << "|" << value << std::endl;
        }
    }
    resultStr = resultTmp.str();
    return resultStr;

}

bool AutomowerSafe::getPitchAndRoll()
{
    hcp_tResult result;
    const char* accelerometerMsg = "RealTimeData.GetSensorData()";
    if (!sendMessage(accelerometerMsg, sizeof(accelerometerMsg), result))
    {
        return false;
    }

    if (result.parameterCount > 5)// == 5)
    {
        int pitch;
        int roll;
        int zAcc;
        unsigned int upside;
        int temperature;
        pitch       = result.parameters[2].value.i16;
        roll        = result.parameters[3].value.i16;
        zAcc      = result.parameters[4].value.i16;
        upside      = result.parameters[5].value.u8;
        temperature = result.parameters[6].value.i16;
        m_pitch = (double)-pitch/10.0 * RADIANS_PER_DEGREE;   // Mower internally use nose up as positive pitch, we use nose down as positive pitch
        m_roll  = (double)roll/10.0 * RADIANS_PER_DEGREE;

        //std::cout << " pitchA: " << pitch << "  roll: " << roll << "  yaw: "  << zAcc << "  upside: " << upside << "   temperature: " << temperature << std::endl;
        //std::cout << "m_pitch: " << m_pitch << "  m_roll: " << m_roll << std::endl;
        return true;
    }
    return false;
}

bool AutomowerSafe::getGPSData()
{
    hcp_tResult result;
    const char* GPS_Msg = "RealTimeData.GetGPSData()";
    if (!sendMessage(GPS_Msg, sizeof(GPS_Msg), result))
    {
        return false;
    }

    if (result.parameterCount == 15)
    {
        uint8_t north;
        uint8_t east;
        unsigned int latitudeDegMinutes;
        unsigned int latitudeDecimalMinute;
        unsigned int longitudeDegMinutes;
        unsigned int longitudeDecimalMinute;
        uint8_t  nbrSatellites;
        double latitude;
        double longitude;
        unsigned int hdop;
        uint8_t GPS_status;

        nbrSatellites          = result.parameters[1].value.u8;
        hdop                   = result.parameters[2].value.u16;
        north                  = result.parameters[3].value.u8;
        east                   = result.parameters[4].value.u8;
        latitudeDegMinutes     = result.parameters[5].value.u32;
        latitudeDecimalMinute  = result.parameters[6].value.u32;
        longitudeDegMinutes    = result.parameters[7].value.u32;
        longitudeDecimalMinute = result.parameters[8].value.u32;
        GPS_status             = result.parameters[14].value.u8;

        if (north == 1)
        {
            latitude = latitudeDegMinutes/100 + (latitudeDegMinutes%100 + latitudeDecimalMinute*0.0001)/60;
        }
        else
        {
            latitude = -(latitudeDegMinutes/100 + (latitudeDegMinutes%100 + latitudeDecimalMinute*0.0001)/60);
        }
        if (east == 1)
        {
            longitude = longitudeDegMinutes/100 + (longitudeDegMinutes%100 + longitudeDecimalMinute*0.0001)/60;
        }
        else
        {
            longitude = -(longitudeDegMinutes/100 + (longitudeDegMinutes%100 + longitudeDecimalMinute*0.0001)/60);
        }

        double covariance = (hdop *1.5)* (hdop *1.5);    // Approximate the covariance

        m_navSatFix_msg.header.stamp  = ros::Time::now();
        m_navSatFix_msg.latitude  = latitude;
        m_navSatFix_msg.longitude = longitude;
        m_navSatFix_msg.altitude  = 0.0;

        for (int i=0; i<9;i++)
        {
            m_navSatFix_msg.position_covariance[i]  = covariance;
        }

        m_navSatFix_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        m_navSatFix_msg.status.status = GPS_status;
        m_navSatFix_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        navSatFix_pub.publish(m_navSatFix_msg);

        //std:: cout << "nSat : " << int(nbrSatellites) << "  lat: " << latitude << "  long: "  << longitude << " status: " << int(GPS_status) << " " << latitudeDegMinutes  << " " << latitudeDecimalMinute << std::endl;
    }
    return true;
}

bool AutomowerSafe::getStateData()
{
    hcp_tResult result;

    //
    // State and Mode check
    //
    const char* msg2 = "MowerApp.GetState()";
    if (!sendMessage(msg2, sizeof(msg2), result))
    {
        ROS_INFO("Couldn't get mower state...");
        return false;
    }
    int state = result.parameters[0].value.u8;
    sensorStatus.mowerInternalState = state;
    switch (state)
    {
    case IMOWERAPP_STATE_OFF:
        sensorStatus.operationalMode = AM_OP_MODE_OFFLINE;
        break;
    case IMOWERAPP_STATE_WAIT_SAFETY_PIN:
        sensorStatus.operationalMode = AM_OP_MODE_OFFLINE;
        break;
    case IMOWERAPP_STATE_STOPPED:
        sensorStatus.operationalMode = AM_OP_MODE_OFFLINE;
        eventQueue->raiseEvent("/AM_STOPPED");
        break;
    case IMOWERAPP_STATE_FATAL_ERROR:
        sensorStatus.operationalMode = AM_OP_MODE_OFFLINE;
        break;
    case IMOWERAPP_STATE_PENDING_START:
        sensorStatus.operationalMode = AM_OP_MODE_OFFLINE;
        break;
    case IMOWERAPP_STATE_PAUSED:
        sensorStatus.operationalMode = AM_OP_MODE_CONNECTED_MANUAL;
        eventQueue->raiseEvent("/AM_PAUSED");
        break;
    case IMOWERAPP_STATE_IN_OPERATION:
        sensorStatus.operationalMode = AM_OP_MODE_CONNECTED_RANDOM;
        eventQueue->raiseEvent("/AM_IN_OPERATION");
        break;

    case IMOWERAPP_STATE_RESTRICTED:
        sensorStatus.operationalMode = AM_OP_MODE_CONNECTED_RANDOM;
        eventQueue->raiseEvent("/AM_IN_OPERATION");
        break;
    case IMOWERAPP_STATE_ERROR:
        sensorStatus.operationalMode = AM_OP_MODE_CONNECTED_RANDOM;
        break;

    default:
        break;
    }

    return true;
}

bool AutomowerSafe::getLoopDetection()
{
    hcp_tResult result;
    //
    // SensorStatus
    //

    const char* loopMsg = "SystemSettings.GetLoopDetection()";
    if (!sendMessage(loopMsg, sizeof(loopMsg), result))
    {
        return false;
    }
    if (result.parameterCount == 1)
    {
        // 1 - Active,
        if (result.parameters[0].value.b == 1)
        {
            sensorStatus.sensorStatus |= HVA_SS_LOOP_ON;
        }
    }

    if (cuttingDiscOn)
    {
        sensorStatus.sensorStatus |= HVA_SS_DISC_ON;
    }

    if (sensorStatus.controlState == AM_STATE_PARK)
    {
        sensorStatus.sensorStatus |= HVA_SS_PARKED;
    }
    return true;
}

bool AutomowerSafe::getStatus()
{
    hcp_tResult result;
    //
    // STOP button
    //
    const char* userStopMsg = "SafetySupervisor.GetStatus()";
    if (!sendMessage(userStopMsg, sizeof(userStopMsg), result))
    {
        ROS_WARN("Can't get Safety supervisor status");
        return false;
    }
    if (result.parameters[0].value.b)
    {
        sensorStatus.sensorStatus |= HVA_SS_USER_STOP;
        userStop = true;
    }
    else
    {
        sensorStatus.sensorStatus &= ~HVA_SS_USER_STOP;

        userStop = false;
    }

    if (result.parameters[2].value.b)
    {
        ROS_INFO("Lifted");
        sensorStatus.sensorStatus |= HVA_SS_LIFTED;
    }
    else
    {
        sensorStatus.sensorStatus &= ~HVA_SS_LIFTED;

        userStop = false;
    }

    if (result.parameters[5].value.b)
    {
        ROS_INFO("Collision");
        sensorStatus.sensorStatus |= HVA_SS_COLLISION;
    }
    else
    {
        sensorStatus.sensorStatus &= ~HVA_SS_COLLISION;

        userStop = false;
    }

    if (result.parameters[12].value.b)
    {
        sensorStatus.sensorStatus |= HVA_SS_CHARGING;
    }
    else
    {
        sensorStatus.sensorStatus &= ~HVA_SS_CHARGING;

        userStop = false;
    }
    return true;
}

bool AutomowerSafe::getChargingPowerConnected()
{
    hcp_tResult result;

    // Check if inside charging station
    const char* msgCPC = "Charger.IsChargingPowerConnected()";
    if (!sendMessage(msgCPC, sizeof(msgCPC), result))
    {
        return false;
    }
    if (result.parameterCount == 1)
    {
        // 1 - Active,
        if (result.parameters[0].value.b == 1)
        {
            sensorStatus.sensorStatus |= HVA_SS_IN_CS;
            userStop = false;
        }
        else
        {
            sensorStatus.sensorStatus &= ~HVA_SS_IN_CS;
            userStop = false;
        }
    }

    return true;
}

bool AutomowerSafe::getStatusKeepAlive()
{
    hcp_tResult result;

    // Send Keep alive message to prevent automower to go to sleep mode
    const char* msgK = "CurrentStatus.GetStatusKeepAlive()";
    if (!sendMessage(msgK, sizeof(msgK), result))
    {
        return false;
    }
    // Send Keep alive message to prevent automower to go to sleep mode
    currentStatus.header.stamp = ros::Time::now();
    currentStatus.state = -1;
    currentStatus.subState = -1;
    currentStatus.mode = -1;
    if (result.parameterCount >= 5)
    {
        currentStatus.state = result.parameters[0].value.u8;
        currentStatus.subState = result.parameters[1].value.u8;
        currentStatus.mode = result.parameters[2].value.u8;
    }

    return true;
}

bool AutomowerSafe::getSensorStatus()
{
    hcp_tResult result;

    sensorStatus.sensorStatus = 0;

    if (!getLoopDetection())
    {
        return false;
    }

    if (!getStatus())
    {
        return false;
    }

    const char* realTimeSensorStatusMsg = "RealTimeData.GetSensorData()";
    if (!sendMessage(realTimeSensorStatusMsg, sizeof(realTimeSensorStatusMsg), result))
    {
        ROS_WARN("Can't get realTimeSensorStatusMsg status");
        return false;
    }
    if (result.parameters[0].value.b)
    {
        ROS_INFO("Collision");
        sensorStatus.sensorStatus |= HVA_SS_COLLISION;
    }
    else
    {
        sensorStatus.sensorStatus &= ~HVA_SS_COLLISION;

        userStop = false;
    }

    if (!getChargingPowerConnected())
    {
        return false;
    }

    // Send Keep alive message to prevent automower to go to sleep mode
    if (!getStatusKeepAlive())
    {
        return false;
    }
    return true;
}

bool AutomowerSafe::getLoopData()
{
    hcp_tResult result;

    //
    // LoopSensor
    //
    const char* loopAmsg = "LoopSampler.GetLoopSignalMaster(loop:0)";
    if (!sendMessage(loopAmsg, sizeof(loopAmsg), result))
    {
        return false;
    }
    if (result.parameterCount == 1)
    {
        // Compability
        loop.frontCenter = result.parameters[0].value.i16;
        loop.frontRight = 0;
        loop.rearLeft = 0;
        loop.rearRight = 0;

        // A-channel
        loop.A0.frontCenter = result.parameters[0].value.i16;
        loop.A0.frontRight = 0;
        loop.A0.rearLeft = 0;
        loop.A0.rearRight = 0;
    }

    const char* loopFmsg = "LoopSampler.GetLoopSignalMaster(loop:1)";
    if (!sendMessage(loopFmsg, sizeof(loopFmsg), result))
    {
        return false;
    }
    if (result.parameterCount == 1)
    {

        // F-channel
        loop.F.frontCenter = result.parameters[0].value.i16;
        loop.F.frontRight = 0;
        loop.F.rearLeft = 0;
        loop.F.rearRight = 0;
    }

    const char* loopNmsg = "LoopSampler.GetLoopSignalMaster(loop:2)";
    if (!sendMessage(loopNmsg, sizeof(loopNmsg), result))
    {
        return false;
    }
    if (result.parameterCount == 1)
    {
        // N-channel
        loop.N.frontCenter = result.parameters[0].value.i16;
        loop.N.frontRight = 0;
        loop.N.rearLeft = 0;
        loop.N.rearRight = 0;
    }

    return true;
}
/*
// An alternative function to the one above. This one gets the value for all the sensors,
// but they might be less up to date than in the one above
bool AutomowerSafe::getLoopData()
{
    hcp_tResult result;

    //
    // LoopSensor
    //

    const char* loopFrontC = "RealTimeData.GetFrontCenterLoopSensorData()";
    if ( !sendMessage( loopFrontC, sizeof( loopFrontC ), result ) )
    {
        return false;
    }

    // Compability
    loop.frontCenter = result.parameters[ 1 ].value.i16;
    loop.A0.frontCenter = result.parameters[ 1 ].value.i16;
    loop.F.frontCenter = result.parameters[ 2 ].value.i16;
    loop.G1.frontCenter = result.parameters[ 3 ].value.i16;
    loop.G2.frontCenter = result.parameters[ 4 ].value.i16;
    loop.G3.frontCenter = result.parameters[ 8 ].value.i16;
    loop.N.frontCenter = result.parameters[ 7 ].value.i16;

    const char* loopFrontR = "RealTimeData.GetFrontRightLoopSensorData()";
    if ( !sendMessage( loopFrontR, sizeof( loopFrontR ), result ) )
    {
        return false;
    }
    // Compability
    loop.frontRight = result.parameters[ 1 ].value.i16;
    loop.A0.frontRight = result.parameters[ 1 ].value.i16;
    loop.F.frontRight = result.parameters[ 2 ].value.i16;
    loop.G1.frontRight = result.parameters[ 3 ].value.i16;
    loop.G2.frontRight = result.parameters[ 4 ].value.i16;
    loop.G3.frontRight = result.parameters[ 8 ].value.i16;
    loop.N.frontRight = result.parameters[ 7 ].value.i16;

    const char* loopRearL = "RealTimeData.GetRearLeftLoopSensorData()";
    if ( !sendMessage( loopRearL, sizeof( loopRearL ), result ) )
    {
        return false;
    }
    // Compability
    loop.rearLeft = result.parameters[ 1 ].value.i16;
    loop.A0.rearLeft = result.parameters[ 1 ].value.i16;
    loop.F.rearLeft = result.parameters[ 2 ].value.i16;
    loop.G1.rearLeft = result.parameters[ 3 ].value.i16;
    loop.G2.rearLeft = result.parameters[ 4 ].value.i16;
    loop.G3.rearLeft = result.parameters[ 8 ].value.i16;
    loop.N.rearLeft = result.parameters[ 7 ].value.i16;

    const char* loopRearR = "RealTimeData.GetRearRightLoopSensorData()";
    if ( !sendMessage( loopRearR, sizeof( loopRearR ), result ) )
    {
        return false;
    }
    // Compability
    loop.rearRight = result.parameters[ 1 ].value.i16;
    loop.A0.rearRight = result.parameters[ 1 ].value.i16;
    loop.F.rearRight = result.parameters[ 2 ].value.i16;
    loop.G1.rearRight = result.parameters[ 3 ].value.i16;
    loop.G2.rearRight = result.parameters[ 4 ].value.i16;
    loop.G3.rearRight = result.parameters[ 8 ].value.i16;
    loop.N.rearRight = result.parameters[ 7 ].value.i16;

    return true;
}
*/
bool AutomowerSafe::getBatteryData()
{
    hcp_tResult result;

    //
    // Battery check
    //
    const char* msg = "RealTimeData.GetBatteryData()";
    if (!sendMessage(msg, sizeof(msg), result))
    {
        return false;
    }

    int16_t batAvoltage = result.parameters[0].value.u16;
    int16_t batAcurrent = result.parameters[2].value.i16;

    int16_t batBvoltage = result.parameters[5].value.u16;
    int16_t batBcurrent = result.parameters[7].value.i16;

    if (printCharge)
    {
        std::cout << "[MOWER] BAT A - Voltage: " << batAvoltage << " mV - Current: " << batAcurrent << " mA" << std::endl;
        std::cout << "[MOWER] BAT B - Voltage: " << batBvoltage << " mV - Current: " << batBcurrent << " mA" << std::endl;
    }

    batteryStatus.header.stamp = ros::Time::now();
    batteryStatus.batteryAVoltage = batAvoltage;
    batteryStatus.batteryACurrent = batAcurrent;
    batteryStatus.batteryBVoltage = batBvoltage;
    batteryStatus.batteryBCurrent = batBcurrent;

    batStatus_pub.publish(batteryStatus);

    return true;
}

void AutomowerSafe::stopWheels()
{

    //DEBUG_LOG ("AutomowerSafe::stopWheels()")
    // Clear the PIDs and power
    leftWheelPid.Restart();
    rightWheelPid.Restart();

    power_l = 0;
    power_r = 0;

//    wheelPower.left = power_l;
//    wheelPower.right = power_r;

    hcp_tResult result;
    const char* powerOffMsg = "Wheels.PowerOff()";
    if (!sendMessage(powerOffMsg, sizeof(powerOffMsg), result))
    {
        return;
    }

    return;
}

void AutomowerSafe::regulateVelocity()
{
    if (!m_regulatingActive)
    {
        return;
    }

    power_l = leftWheelPid.Update(current_lv, wanted_lv);
    power_r = rightWheelPid.Update(current_rv, wanted_rv);

    sendWheelPower(power_l, power_r);
}

void AutomowerSafe::setPower()
{
    if (!m_regulatingActive)
    {
        return;
    }

    power_l = wanted_power_left;
    power_r = wanted_power_right;

    sendWheelPower(power_l, power_r);
}

void AutomowerSafe::sendWheelPower(double power_left, double power_right)
{
    if (userStop)
    {
        ROS_WARN("User stop active, can't set power");
        stopWheels();
        return;
    }

    if (power_l > 100)
    {
        power_l = 100;
    }

    if (power_l < -100)
    {
        power_l = -100;
    }

    if (power_r > 100)
    {
        power_r = 100;
    }

    if (power_r < -100)
    {
        power_r = -100;
    }

    // Send it out...
    hcp_tResult result;
    char powerMsg[100];
    snprintf(powerMsg, sizeof(powerMsg), "HardwareControl.WheelMotorsPower(leftWheelMotorPower:%d, rightWheelMotorPower:%d)", power_l, power_r);
    if (!sendMessage(powerMsg, sizeof(powerMsg), result))
    {
        ROS_WARN("Can't set power, unknown reason");
        return;
    }
}



bool AutomowerSafe::doSerialComTest()
{
    hcp_tResult result;
    int i;

    if (printCharge)
    {
        const char* msg = "RealTimeData.GetBatteryData()";
        if (!sendMessage(msg, sizeof(msg), result))
        {
            return false;
        }

        int16_t batAvoltage = result.parameters[0].value.u16;
        int16_t batAcurrent = result.parameters[2].value.i16;

        int16_t batBvoltage = result.parameters[5].value.u16;
        int16_t batBcurrent = result.parameters[7].value.i16;

        std::cout << "[MOWER] BAT A - Voltage: " << batAvoltage << " mV - Current: " << batAcurrent << " mA" << std::endl;
        std::cout << "[MOWER] BAT B - Voltage: " << batBvoltage << " mV - Current: " << batBcurrent << " mA" << std::endl;
    }


    // Send Keep alive message to prevent automower to go to sleep mode
    const char* msgK = "CurrentStatus.GetStatusKeepAlive()";
    if (!sendMessage(msgK, sizeof(msgK), result))
    {
        return false;
    }

    char msgA[100];
    snprintf(msgA, sizeof(msgA), "HardwareControl.WheelMotorsPower(leftWheelMotorPower:%d, rightWheelMotorPower:%d)", lastComtestWheelMotorPower, -lastComtestWheelMotorPower);
    const char* msgB = "Wheels.GetSpeed(index:1)";

    for (i = 0; i  < 50; i++)
    {
        if (!sendMessage(msgA, sizeof(msgA), result))
        {
            return false;
        }

        if (!sendMessage(msgB, sizeof(msgB), result))
        {
            return false;
        }

    }

    if (lastComtestWheelMotorPower < 25)
    {
        lastComtestWheelMotorPower += 2;
    }

    return true;

}

bool AutomowerSafe::isTimeOut(ros::Duration elapsedTime, double frequency)
{
    if (fabs(frequency) < 1e-6)
    {
        return false;
    }
    else if (elapsedTime >= ros::Duration(1.0/frequency))
    {
        return true;
    }

    return false;
}

/**
 * Failing to send will reset state machine to zero
 * Sucess will increment state
 * */
void AutomowerSafe::sendGuideCommand(std::string cmd, double delaySec, ros::Duration dt)
{

    guideTimer += dt;
    if (guideTimer.toSec() > delaySec)
    {
        hcp_tResult result;
        const char* msg = cmd.c_str();
        if (!sendMessage(msg, sizeof(msg), result))
        {
            eventQueue->raiseEvent("/COM_ERROR");
            ROS_WARN("sendGuideCommand failed!");
            followGuideState = 0;
        }
        
        ROS_INFO("%d: Sending: %s",followGuideState, msg);
        std::string strRes;
        strRes = resultToString(result);
        std::cout << strRes << std::endl;

        guideTimer = ros::Duration(0.0);
        ++followGuideState;
    }
}

void AutomowerSafe::handleFollowGuide(ros::Duration dt)
{
    std::stringstream message;
    char buf[200];
    switch (followGuideState)
    {   
        //Dont do anything until triggered
        case 0:
            break;


        case 1: //Simulate Stop Button
        { 
            sendGuideCommand("StopButton.SetSimulation(simOnOff:1)", 0.1, dt);
            break;
        }

        case 2: // Press stop
        {
            sendGuideCommand("StopButton.SetSimValue(switchOnOff:1)", 0.1, dt);
            break;
        }

        case 3: // Enter RANDOM mode
        {
            setRandomMode();
            ++followGuideState;
            break;
        }

        case 4: //Disable GPS
        {
            sendGuideCommand("DrivingSettings.SetGpsSettings(GPSNavigationEnable:0)", 0.1, dt);
            break;
        }

        case 5: //Configure width (driving over guide for best accuracy (?))
        {
            
            sprintf(buf,    "DrivingSettings.SetCorridor(loopWire:%d, minDistToWire:%d,     maxDistToWire:%d,   wireInGuideCorridor:%d, autoDistanceEnable:%d)", 
                                                        wireToFollow, inCorridorMinWidth,   inCorridorMaxWidth, wireInGuideCorridor,    autoDistanceEnabled );
            sendGuideCommand(std::string(buf), 0.1, dt);
            break;
        }
        case 6: //Configure the properties (follow G1, no delay)
        {
            sprintf(buf, "DrivingSettings.SetFollowWireIn(  loopWire:%d, delayTime:%d, followWireInEnable:%d)",
                                                            wireToFollow, delayTime, followWireInEnable);
            sendGuideCommand(std::string(buf), 0.1, dt);
            break;
        }
        case 7: //Activate following
        {
            sprintf(buf, "DrivingSettings.SetTestFollowWireIn(loopWire:%d)", wireToFollow);
            sendGuideCommand(std::string(buf), 0.3, dt);
            break;
        }        
        case 8: //Set F-field as week as possible
        {
            sprintf(buf,"DrivingSettings.SetCSRange(range:%d)", CS_Range_min);
            sendGuideCommand(std::string(buf), 0.1, dt);
            break;
        }
        case 9: //Issue the command
        {
            sendGuideCommand("MowerCommands.TestFollowIn()", 0.2, dt);
            break;
        }

        case 10: //Do the start sequence
        {
            sendGuideCommand("MowerCommands.PrepareStart()", 0.3, dt);
            break;
        }

        case 11: //Close it the hatch
        {
            sendGuideCommand("StopButton.SetSimValue(switchOnOff:0)", 0.3, dt);
            break;
        }

        case 12: //Stop simulating the hatch
        {
            sendGuideCommand("StopButton.SetSimulation(simOnOff:0)", 0.1, dt);
            break;
        }
        case 13: //Done
        {
            followGuideState = 0;
            break;
        }

        //Abort the following, and enter manual mode------------
        
        case 255: //Start simulating the hatch
        {
            sendGuideCommand("StopButton.SetSimulation(simOnOff:1)", 0.3, dt);
            break;
        }

        case 256: //Open the hatch
        {
            sendGuideCommand("StopButton.SetSimValue(switchOnOff:1)", 0.3, dt);
            break;
        }

        case 257: //Command stop following
            sendGuideCommand("MowerCommands.StopFollowWire()", 0.3, dt);
            break;

        case 258: // Enter MANUAL mode
        {
            setManualMode();
            ++followGuideState;
            break;
        }

        case 259: //Do the start sequence
        {
            sendGuideCommand("MowerCommands.PrepareStart()", 0.3, dt);
            break;
        }

        case 260: //Close the hatch
        {
            sendGuideCommand("StopButton.SetSimValue(switchOnOff:0)", 0.3, dt);
            break;
        }

        case 261: //Stop simulating the hatch
        {
            sendGuideCommand("StopButton.SetSimulation(simOnOff:0)", 0.3, dt);
            break;
        }

        case 262: //Set F-field as strong as possible
        {
            sprintf(buf,"DrivingSettings.SetCSRange(range:%d)", CS_Range_max);
            sendGuideCommand(std::string(buf), 0.1, dt);
            break;
        }

        case 263: //Done
        {
            followGuideState = 0;
            break;
        }

        default:
            followGuideState = 0;
            break;
    }

    return;
}


void AutomowerSafe::handleCollisionInjections(ros::Duration dt)
{
    switch (collisionState)
    {
        // 1 - 4: states for manually injecting collision
        case 1:
        {
            ROS_INFO("Collision.SetSimulation(onOff:1)");
            hcp_tResult result; 
            const char* msg = "Collision.SetSimulation(onOff:1)";
            if (!sendMessage(msg, sizeof(msg), result))
            {
                eventQueue->raiseEvent("/COM_ERROR");
            }
            collisionState = 2;
            break;
        }
        case 2:
        {
            ROS_INFO("Collision.SetSimulatedStatus(status:1)");
            hcp_tResult result; 
            const char* msg =  "Collision.SetSimulatedStatus(status:1)";
            if (!sendMessage(msg, sizeof(msg), result))
            {
                eventQueue->raiseEvent("/COM_ERROR");
            }
            timeSinceCollision = ros::Duration(0.0);
            collisionState = 3;
            break;
        }
        case 3:
        {   
            
            timeSinceCollision += dt;
            // x ms before deactivating the collision signal
            if (isTimeOut(timeSinceCollision, 1))
            {
                ROS_INFO( "Collision.SetSimulatedStatus(status:0)");
                hcp_tResult result; 
                const char* msg =  "Collision.SetSimulatedStatus(status:0)";
                if (!sendMessage(msg, sizeof(msg), result))
                {
                    eventQueue->raiseEvent("/COM_ERROR");
                }
                collisionState = 4;
            }
            break;
            
        }
        case 4:
        {
            hcp_tResult result; 
            ROS_INFO("Collision.SetSimulation(onOff:0)");
            const char* msg = "Collision.SetSimulation(onOff:0)";
            if (!sendMessage(msg, sizeof(msg), result))
            {
                eventQueue->raiseEvent("/COM_ERROR");
            }
            collisionState = 0;
            break;
        }


        //state 5 - 6  - Disable collision sensors
        case 5:
        {
            ROS_INFO("Collision.SetSimulation(onOff:1)");
            hcp_tResult result; 
            const char* msg = "Collision.SetSimulation(onOff:1)";
            if (!sendMessage(msg, sizeof(msg), result))
            {
                eventQueue->raiseEvent("/COM_ERROR");
            }
            collisionState = 6;
            break;
        }
        case 6:
        {
            ROS_INFO("Collision.SetSimulatedStatus(status:0)");
            hcp_tResult result; 
            const char* msg =  "Collision.SetSimulatedStatus(status:0)";
            if (!sendMessage(msg, sizeof(msg), result))
            {
                eventQueue->raiseEvent("/COM_ERROR");
            }
            collisionState = 0;
            break;
        }

        //state 7  - Enable collision sensors
        case 7:
        {
            ROS_INFO("Collision.SetSimulation(onOff:0)");
            hcp_tResult result; 
            const char* msg = "Collision.SetSimulation(onOff:0)";
            if (!sendMessage(msg, sizeof(msg), result))
            {
                eventQueue->raiseEvent("/COM_ERROR");
            }
            collisionState = 0;
            break;
        }

    }
}

bool AutomowerSafe::update(ros::Duration dt)
{
    if (serialPortState == AM_SP_STATE_ERROR)
    {
        if (!serialComTest)
        {
            nextAutomowerInitTime = ros::Time::now() + ros::Duration(2.0);  // only wait a short time
            ROS_WARN("Communication error. New try in 2 seconds");
        }
        else
        {
            //In this case we want a long pause to see the automower behaviour
            nextAutomowerInitTime = ros::Time::now() + ros::Duration(30);   // Results in waiting in 30 second
            ROS_WARN("SerialComTest:   Communication error. New try in 30 seconds");
        }
        serialPortState = AM_SP_STATE_OFFLINE;
    }

    if (serialPortState == AM_SP_STATE_OFFLINE)
    {

        if (ros::Time::now() > nextAutomowerInitTime)
        {
            serialPortState = AM_SP_STATE_INITIALISING;
            if (initAutomowerBoard())
            {

                ROS_INFO("Automower::Serial port ONLINE!");
                serialPortState = AM_SP_STATE_ONLINE;
            }
            else
            {
                serialPortState = AM_SP_STATE_OFFLINE;
                nextAutomowerInitTime = ros::Time::now() + ros::Duration(10);  // retry every 10 second
                ROS_WARN("Automower::Failed to contact Mower board - SLEEPING?");
                ROS_WARN("New try in 10 seconds");
            }
        }
        else
        {
            ROS_INFO("WAITING\b");
        }

    }

    if (serialPortState == AM_SP_STATE_ONLINE)
    {
        ROS_INFO("Automower::Serial port connected!");
        serialPortState = AM_SP_STATE_CONNECTED;


        if (requestedState == AM_STATE_MANUAL)
        {
            eventQueue->raiseEvent("/MANUAL");
        }
        else if (requestedState == AM_STATE_RANDOM)
        {
            eventQueue->raiseEvent("/RANDOM");
        }
    }

    if (serialComTest)
    {
        doSerialComTest();
        return true;
    }

    ros::Time current_time = ros::Time::now();

    timeSinceWheelSensor += dt;
    timeSinceEncoderSensor += dt;
    timeSinceRegulator += dt;
    timeSinceState += dt;
    timeSinceLoop += dt;
    timeSincePitchRoll += dt;
    timeSincebattery += dt;
    timeSinceGPS += dt;
    timeSinceStatus += dt;

    bool newData = false;

    // Communication with mower if serial port connected
    if (serialPortState == AM_SP_STATE_CONNECTED )
    {
        if (isTimeOut(timeSinceWheelSensor, wheelSensorFreq))
        {
            getWheelData();
//            double rate = 1.0/timeSinceWheelSensor.toSec();
//            DEBUG_LOG("Actual Wheelspeed rate: " << rate);
            timeSinceWheelSensor *= 0;
            newData = true;

        }
        if (isTimeOut(timeSinceEncoderSensor, encoderSensorFreq))
        {
            getEncoderData();
//            double rate = 1.0/timeSinceEncoderSensor.toSec();
//            DEBUG_LOG("Actual encoderSensor rate: " << rate);
            timeSinceEncoderSensor *= 0;
            newData = true;

        }
        if (isTimeOut(timeSinceRegulator, regulatorFreq))
        {
            if (velocityRegulator)
            {
                regulateVelocity();
//                double rate = 1.0/timeSinceRegulator.toSec();
//                DEBUG_LOG("Actual Regulation rate: " << rate);
                timeSinceRegulator *= 0;
                newData = true;
            }
            else
            {
                setPower();
//                double rate = 1.0/timeSinceRegulator.toSec();
//                DEBUG_LOG("Actual setPower rate: " << rate);
                timeSinceRegulator *= 0;
                newData = true;
            }
        }
        if (isTimeOut(timeSinceStatus, sensorStatusCheckFreq))
        {
            getSensorStatus();
            timeSinceStatus *= 0;
            newData = true;
        }
        if (isTimeOut(timeSinceState, stateCheckFreq))
        {
            getStateData();
            timeSinceState *= 0;
            newData = true;
        }
        if (isTimeOut(timeSinceLoop, loopSensorFreq))
        {
            getLoopData();
            timeSinceLoop *= 0;
            newData = true;
        }
        if (m_PitchAndRollFromAccelerometer &&
                 isTimeOut(timeSincePitchRoll, pitchRollFreq))
        {
            getPitchAndRoll();
            timeSincePitchRoll *= 0;
            newData = true;
        }
        if (isTimeOut(timeSincebattery, batteryCheckFreq))
        {
            getBatteryData();
            timeSincebattery *= 0;
            newData = true;
        }
        if (isTimeOut(timeSinceGPS, GPSCheckFreq))
        {
            getGPSData();
            timeSinceGPS *= 0;
            newData = true;
        }

        if (newSound)
        {
            hcp_tResult result; 
            const char* msg = soundCmd;
            if (!sendMessage(msg, sizeof(msg), result))
            {
                eventQueue->raiseEvent("/COM_ERROR");
            }
            newSound = false;
        }

        handleCollisionInjections(dt);
        handleFollowGuide(dt);
    }

    if (!newData)
    {
        return true;
    }

    if (timeSincePitchRoll.toSec() < 1e-6)
    {
        if (publishEuler)
        {
            geometry_msgs::Vector3Stamped tmp;
            tmp.header.stamp = ros::Time::now();
            tmp.vector.x = m_roll;
            tmp.vector.y = m_pitch;
            tmp.vector.z = 0;
            euler_pub.publish(tmp);
        }
    }

    if (timeSinceWheelSensor.toSec() < 1e-6)
    {
        motorFeedbackDiffDrive_pub.publish(motorFeedbackDiffDrive);
    }

    if (timeSinceEncoderSensor.toSec() < 1e-6)
    {

        // Get the odo data and convert to meters
        int deltaLeftPulses = leftPulses - lastLeftPulses;
        int deltaRightPulses = rightPulses - lastRightPulses;
        lastLeftPulses = leftPulses;
        lastRightPulses = rightPulses;

        double leftDist = -deltaLeftPulses * WHEEL_METER_PER_TICK;
        double rightDist = deltaRightPulses * WHEEL_METER_PER_TICK;

        // handle ZEROING of counters from mower?
        if ((fabs(leftDist) > 1.0) || (fabs(rightDist) > 1.0))
        {
            DEBUG_LOG("Automower::Strange distance? - zeroint counters from mower");
            // ROS_INFO("Automower::Strange distance? => ld = %f, rd = %f", leftDist, rightDist);
            leftDist = 0;
            rightDist = 0;
        }

        // ROS_INFO("Automower::ld = %f, rd = %f", leftDist, rightDist);

        double distance = (rightDist + leftDist) / 2.0;
        double delta_yaw = -(leftDist - rightDist) / AUTMOWER_WHEEL_BASE_WIDTH;

        yaw = yaw + delta_yaw;

        double vYaw = (yaw - last_yaw) / dt.toSec();
        last_yaw = yaw;

        double xdist = distance * cos(yaw);
        double ydist = distance * sin(yaw);

        double vx = distance / dt.toSec();
        double vy = 0.0;

        xpos = xpos + xdist;
        ypos = ypos + ydist;

        // ROS_INFO("Automower::pos: dt=%f xpos=%f, ypos=%f", (dt.toSec()), (float)xpos, (float)ypos);

        // Set position into the pose
        robot_pose.pose.position.x = xpos;
        robot_pose.pose.position.y = ypos;


		// Calculate the orientation
        tf::Quaternion qyaw;

        if (m_PitchAndRollFromAccelerometer)
        {
            qyaw = tf::createQuaternionFromRPY(m_roll,m_pitch, yaw); 
        }
        else
        {
            qyaw = tf::createQuaternionFromYaw(yaw);
        }

        // Set orientation into the pose

        robot_pose.pose.orientation.x = qyaw.x();
        robot_pose.pose.orientation.y = qyaw.y();
        robot_pose.pose.orientation.z = qyaw.z();
        robot_pose.pose.orientation.w = qyaw.w();


        // Publish wheel encoders

        encoder.lwheel = leftDist; //current_lv;
        encoder.rwheel = rightDist; //current_rv;
        encoder.lwheelAccum = leftPulses;
        encoder.rwheelAccum = rightPulses;
        encoder.lticks = current_lv * 1000.0; //deltaLeftPulses;
        encoder.rticks = current_rv * 1000.0; //deltaRightPulses;

        // std::cout << "LeftDist: " << leftDist << " RightDist: " << rightDist;
        // std::cout << " LeftAccum: " << (int)leftPulses << " RightAccum: " << (int)rightPulses << std::endl;
        encoder_pub.publish(encoder);
        current_pub.publish(wheelCurrent);
        wheelPower_pub.publish(wheelPower);

        // Calculate and Send the TF
        if (publishTf)
        {
            // Calculate the TF from the pose...
			transform.setOrigin(
						tf::Vector3(robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z));
			tf::Quaternion q;
			tf::quaternionMsgToTF(robot_pose.pose.orientation, q);
			transform.setRotation(q);

            br.sendTransform(tf::StampedTransform(transform, current_time, odomFrame, baseLinkFrame));
        }

        // Odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = odomFrame;
        odom.child_frame_id = baseLinkFrame;

        // Set the position
        odom.pose.pose.position.x = robot_pose.pose.position.x;
        odom.pose.pose.position.y = robot_pose.pose.position.y;
        odom.pose.pose.position.z = robot_pose.pose.position.z;
        odom.pose.pose.orientation = robot_pose.pose.orientation;

        // Set the velocity
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vYaw;

        // Publish the message
        odom_pub.publish(odom);

        // Publish the pose
        pose_pub.publish(robot_pose);
    }

    if (timeSinceLoop.toSec() < 1e-6)
    {
        // Publish the loop
        loop.header.stamp = current_time;
        loop_pub.publish(loop);
    }
    if (timeSinceState.toSec() < 1e-6)
    {
        // Publish the sensorStatus
        if (serialPortState != AM_SP_STATE_CONNECTED)
        {
            // We are offline (i.e. standby?)
            sensorStatus.operationalMode = AM_OP_MODE_OFFLINE;
        }

        sensorStatus.header.stamp = current_time;
        sensorStatus.header.frame_id = odomFrame;
        sensorStatus_pub.publish(sensorStatus);

        currentStatus.header.stamp = current_time;
        currentStatus.header.frame_id = odomFrame;
        currentStatus_pub.publish(currentStatus);
    }

    return true;
}

void AutomowerSafe::pauseMower()
{
    DEBUG_LOG("AutoMowerSafe::pauseMower()");

    hcp_tResult result;

    const char* msg = "MowerApp.Pause()";
    if (!sendMessage(msg, sizeof(msg), result))
    {
        eventQueue->raiseEvent("/COM_ERROR");
    }
}
void AutomowerSafe::startMower()
{
    DEBUG_LOG("AutoMowerSafe::startMower()");

    hcp_tResult result;
    const char* msg = "MowerApp.StartTrigger()";
    if (!sendMessage(msg, sizeof(msg), result))
    {
        eventQueue->raiseEvent("/COM_ERROR");
    }
}
void AutomowerSafe::setParkMode()
{
    DEBUG_LOG("AutoMowerSafe::parkMower()");

    hcp_tResult result;
    char msg[100];
    snprintf(msg, sizeof(msg), "MowerApp.SetMode(modeOfOperation:%d)",IMOWERAPP_MODE_HOME);

    if (!sendMessage(msg, sizeof(msg), result))
    {
        eventQueue->raiseEvent("/COM_ERROR");
    }
}
void AutomowerSafe::setAutoMode()
{
    DEBUG_LOG("AutoMowerSafe::setAutoMode()");

    hcp_tResult result;
    char msg[100];
    snprintf(msg, sizeof(msg), "MowerApp.SetMode(modeOfOperation:%d)",IMOWERAPP_MODE_AUTO);

    if (!sendMessage(msg, sizeof(msg), result))
    {
        eventQueue->raiseEvent("/COM_ERROR");
    }
}
void AutomowerSafe::cutDiscHandling()
{
    DEBUG_LOG("AutoMowerSafe::cutDiscHandling()");

    hcp_tResult result;
    if (cuttingDiscOn)
    {
        const char* msg = "BladeMotor.On()";
        if (!sendMessage(msg, sizeof(msg), result))
        {
            eventQueue->raiseEvent("/COM_ERROR");
            return;
        }
        const char* msg1 = "BladeMotor.Run()";
        if (!sendMessage(msg1, sizeof(msg1), result))
        {
            eventQueue->raiseEvent("/COM_ERROR");
            return;
        }

    }
    else
    {
        cutDiscOff();
    }
}

void AutomowerSafe::loopDetectionHandling()
{
    DEBUG_LOG("AutoMowerSafe::loopDetectionHandling()" );

    hcp_tResult result;

    char msg[100];
    snprintf(msg, sizeof(msg), "SystemSettings.SetLoopDetection(loopDetection:%d)", requestedLoopOn);
    if (!sendMessage(msg, sizeof(msg), result))
    {
        ROS_ERROR("Automower::Failed setting LoopDetection on/off");
        eventQueue->raiseEvent("/COM_ERROR");
    }

}


void AutomowerSafe::cuttingHeightHandling()
{
    hcp_tResult result;
    DEBUG_LOG("AutoMowerSafe::cuttingHeightHandling()" );
    ROS_INFO("Automower::set new cutting height= %d mm", cuttingHeight);

    char msg[100];
    snprintf(msg, sizeof(msg), "HeightMotor.SetHeight(height:%d)", cuttingHeight);

    if (!sendMessage(msg, sizeof(msg), result))
    {
        ROS_ERROR("Automower::Failed setting cutting height.");
        cuttingHeight = lastCuttingHeight;
        eventQueue->raiseEvent("/COM_ERROR");
    }

}


void AutomowerSafe::cutDiscOff()
{
    hcp_tResult result;
    DEBUG_LOG("AutoMowerSafe::cutDiscOff()" );

    const char* msg = "BladeMotor.Brake()";
    if (!sendMessage(msg, sizeof(msg), result))
    {
        eventQueue->raiseEvent("/COM_ERROR");
    }

}

void AutomowerSafe::newControlMainState(int aNewState)
{
    sensorStatus.controlState = aNewState;
}

int AutomowerSafe::GetUpdateRate()
{
    return updateRate;
}

void AutomowerSafe::clearOverRide()
{
    hcp_tResult result;
    DEBUG_LOG("AutoMowerSafe::clearOverride()" );

    const char* msg = "Planner.ClearOverride()";

    if (!sendMessage(msg, sizeof(msg), result))
    {
        eventQueue->raiseEvent("/COM_ERROR");
    }

}
	




}
