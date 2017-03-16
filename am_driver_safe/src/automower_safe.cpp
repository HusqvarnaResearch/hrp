/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#include "am_driver_safe/automower_safe.h"
#include <tf/transform_datatypes.h>

#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <fstream>
#include <sstream>

#define DEG2RAD(DEG) ((DEG) * ((M_PI) / (180.0)))

#define MODE_UNDEFINED 0x0000
#define MODE_MANUAL 0x0001
#define MODE_RANDOM 0x0002
#define MODE_PARK 0x0003
#define MODE_STOPPED 0x0004

// OPERATIONAL MODES (i.e. published in SensorStatus)
#define AM_OP_MODE_OFFLINE (0x0000)
#define AM_OP_MODE_CONNECTED_MANUAL (0x0001)
#define AM_OP_MODE_CONNECTED_RANDOM (0x0002)

#define AM_SP_STATE_OFFLINE (0)
#define AM_SP_STATE_ONLINE (1)
#define AM_SP_STATE_CONNECTED (2)
#define AM_SP_STATE_INITIALISING (3)
#define AM_SP_STATE_ERROR (4)

#define AM_MAINBOARD_NEEDS_CONFIG (0)
#define AM_MAINBOARD_ROS_IN_CONTROL (1)
#define AM_MAINBOARD_HMB_IN_CONTROL (2)

#define AM_POWER_INCDEC (1)
#define AM_POWER_THRESHOLD (0.1)

#define HVA_SS_HMB_CTRL 0x0001
#define HVA_SS_OUTSIDE 0x0002
#define HVA_SS_COLLISION 0x0004
#define HVA_SS_LIFTED 0x0008
#define HVA_SS_TOO_STEEP 0x0010
#define HVA_SS_PARKED 0x0020
#define HVA_SS_IN_CS 0x0040
#define HVA_SS_USER_STOP 0x0080
#define HVA_SS_CFG_NEEDED 0x0100
#define HVA_SS_DISC_ON 0x0200
#define HVA_SS_LOOP_ON 0x0400

#define IMOWERAPP_MODE_AUTO (0)
#define IMOWERAPP_MODE_MANUAL (1)
#define IMOWERAPP_MODE_HOME (2)
#define IMOWERAPP_MODE_DEMO (3)


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

AutomowerSafe::AutomowerSafe(const ros::NodeHandle& nodeh, double anUpdateRate)
{
    // Init attributes
    nh = nodeh;
    updateRate = anUpdateRate;
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

    // Check if we shall publish the TF's?
    // It could be our "position" node that should do this...
    n_private.param("publishTf", publishTf, 1);
    ROS_INFO("Param: publishTf: [%d]", publishTf);

    n_private.param("velocityRegulator", velocityRegulator, 1);
    ROS_INFO("Param: velocityRegulator: [%d]", velocityRegulator);

    n_private.param("wheelDiameter", WHEEL_DIAMETER, 0.245);
    n_private.param("wheelBase", AUTMOWER_WHEEL_BASE_WIDTH, 0.464500);

    n_private.param("printCharge", printCharge, true);
    ROS_INFO("Param: printCharge: [%d]", printCharge);

    n_private.param("serialComTest", serialComTest, false);
    ROS_INFO("Param: serialComTest: [%d]", serialComTest);
    
    n_private.param("serialLog",serialLog , false);
    ROS_INFO("Param: serialLog: [%d]", serialLog);
    

    // Setup some ROS stuff
    cmd_sub = nh.subscribe("cmd_mode", 5, &AutomowerSafe::modeCallback, this);


    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

    encoder_pub = nh.advertise<am_driver::WheelEncoder>("wheel_encoder", 1);
    current_pub = nh.advertise<am_driver::WheelCurrent>("wheel_current", 1);
    wheelPower_pub = nh.advertise<am_driver::WheelPower>("wheel_power", 1);

    imuResetSub = nh.subscribe("imu_reset", 1, &AutomowerSafe::imuResetCallback, this);
    velocity_sub = nh.subscribe("cmd_vel", 1, &AutomowerSafe::velocityCallback, this);

    loop_pub = nh.advertise<am_driver::Loop>("loop", 1);
    sensorStatus_pub = nh.advertise<am_driver::SensorStatus>("sensor_status", 1);
    batStatus_pub  = nh.advertise<am_driver::BatteryStatus>("battery_status", 1);

    tifCommandService = nh.advertiseService("tif_command", &AutomowerSafe::executeTifCommand, this);
    ROS_INFO("Service /tif_command.");

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
    mode = MODE_MANUAL;
    requestedMode = MODE_MANUAL;

    tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
    robot_pose.pose.orientation.x = q.x();
    robot_pose.pose.orientation.y = q.y();
    robot_pose.pose.orientation.z = q.z();
    robot_pose.pose.orientation.w = q.w();

    robot_pose.header.frame_id = "base_link";
    robot_pose.header.stamp = ros::Time::now();

    imuOffset = 0.0;

    cuttingDiscOn = false;
    lastCuttingDiscOn = false;

    cuttingHeight = 0;  // Not calibrated, undefined
    lastCuttingHeight = 0; // Not calibrated, undefined
    
    loopOn = true;
    requestedLoopOn = true;

    serialPortState = AM_SP_STATE_OFFLINE;

    lastComtestWheelMotorPower = 3;
    
    actionResponse = 0;

    std::string tmp;
    //nh.param("jsonFile", tmp, (std::string) "./config/31.7_P2_Main_App-Certified_master_build_285-Debug.json");
    n_private.param("jsonFile", tmp, (std::string) "./config/31.7_Main-App-P2_master_build-542_Debug.json");
    jsonFile = tmp;

    power_l = 0;
    power_r = 0;
    batteryCheckCounter = 1;
    sensorCheckCounter = 2;
    loopSensorCheckCounter = 3;
    stateCheckCounter = 4;
    sendStopCuttingDiscCounter = 0;

    nextAutomowerInitTime = ros::Time::now();
    startTime = ros::WallTime::now().toSec();

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

    error = hcp_LoadModel(hcpState, (hcp_szStr)model.c_str(), sizeof(model.c_str()), &modelId);
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

	active = true;

    ROS_INFO("AutomowerSafe::Loaded HCP/TIF codec...let's go!");


}

AutomowerSafe::~AutomowerSafe()
{
    if (serialFd >= 0)
    {
        close(serialFd);
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

    ROS_INFO("Automower::setup()");

    // Open serial port
    serialFd = open(pSerialPort.c_str(), O_RDWR /*| O_NONBLOCK */);
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
    term.c_cc[VMIN] = 0;
    term.c_cc[VTIME] = 20;
    tcsetattr(serialFd, TCSANOW, &term);

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
    last_command_time = ros::WallTime::now();
    lin_vel = (double)vel->linear.x;
    ang_vel = (double)vel->angular.z;

    wanted_lv = lin_vel - ang_vel * AUTMOWER_WHEEL_BASE_WIDTH / 2;
    wanted_rv = lin_vel + ang_vel * AUTMOWER_WHEEL_BASE_WIDTH / 2;

    // ROS_INFO("Automower::cmd_vel: %f m/s  %f rad/s => wanted_lv=%f, wanted_rv=%f", (float)lin_vel, (float)ang_vel,
    // (float)wanted_lv, (float)wanted_rv);
}

void AutomowerSafe::modeCallback(const std_msgs::UInt16::ConstPtr& msg)
{
    if (msg->data < 0x90)
    {
        // Not for us...
        return;
    }

    if (msg->data == 0x90)
    {
        requestedMode = MODE_MANUAL;
        ROS_INFO("Manual Mode Requested");
    }
    else if (msg->data == 0x91)
    {
        requestedMode = MODE_RANDOM;
        ROS_INFO("Random Mode Requested");
    }
    else if (msg->data == 0x92)
    {
        cuttingDiscOn = false;
        ROS_INFO("Cutting Disc OFF");
    }
    else if (msg->data == 0x93)
    {
        cuttingDiscOn = true;
        ROS_INFO("Cutting Disc ON");
    }
    else if (msg->data == 0x94)
    {
        cuttingHeight = 60;
        ROS_INFO("Cutting Height = 60mm");
        
    }
    else if (msg->data == 0x95)
    {
        cuttingHeight = 40;
        ROS_INFO("Cutting Height = 40mm");
        
    }
    else if (msg->data == 0x100)
    {
		requestedMode = MODE_PARK;
        ROS_INFO("Parking");
    }
    else if (msg->data == 0x110)
    {
		requestedLoopOn = true;
        ROS_INFO("Lopp off");
    }
    else if (msg->data == 0x111)
    {
		requestedLoopOn = false;
        ROS_INFO("Loop on");
    }
    else
    {
        // Do nothing...probably not for me...
        return;
    }
}

bool AutomowerSafe::sendMessage(const char* msg, int len, hcp_tResult& result)
{
    double endSendTime;

    if ((serialPortState == AM_SP_STATE_OFFLINE) || (serialPortState == AM_SP_STATE_ERROR))
    {
        return false;
    }

    hcp_Uint8 buf[255];
    hcp_Int numBytes = 0;

    numBytes = hcp_Encode(hcpState, codecId, (hcp_szStr)msg, buf, 255);

    if (serialLog) { 
	    std::cout << "SEND: " << std::hex;
	    for (int i=0; i<numBytes; i++)
	    {
		    std::cout << std::hex << (int)buf[i] << " ";
	    }
	    std::cout << std::dec;
    }

    int cnt = 0;
    cnt = write(serialFd, buf, numBytes);

    if (cnt != numBytes)
    {
        ROS_ERROR("Automower::Could not send on serial port!");
        serialPortState = AM_SP_STATE_ERROR;
        return false;
    }
    endSendTime = ros::WallTime::now().toSec()-startTime;

    numBytes = 0; // to be assigned after read again
    cnt = 0;
    int res;
    int payloadLength = 0;

    // Clear answer buffer (only using one byte...but reused from above)
    memset(buf, 0, 255);

    memset(&result, 0, sizeof(result));

    // Read one byte
    res = read(serialFd, &buf[0], 1);

    double responseTime = ros::WallTime::now().toSec() - startTime - endSendTime;
    


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
            return false;
        }
    }

    if (serialLog) {  std::cout << std::dec << std::endl; }

    if (res <= 0)
    {
        ROS_WARN("Automower::Failed to get response...sleeping?");
        serialPortState = AM_SP_STATE_ERROR;

        return false;
    }

    if (result.error != HCP_NOERROR)
    {
        ROS_WARN("Automower::Error receiving...not logged in?");
        return false;
    }

    
 
    //std::cout << "SAFE::result.parameterCount= " << result.parameterCount << std::endl;

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

    if (mowerType == 7)
    {
        // Get some stuff out from the mower...
        //WHEEL_DIAMETER = ;

        WHEEL_PULSES_PER_TURN = 349;
        WHEEL_METER_PER_TICK = (2.0 * M_PI * WHEEL_DIAMETER / 2.0) / (double)WHEEL_PULSES_PER_TURN;
    }

    ROS_INFO("Automower::WHEEL_DIAMETER = %f", WHEEL_DIAMETER);
    ROS_INFO("Automower::AUTMOWER_WHEEL_BASE_WIDTH = %f", AUTMOWER_WHEEL_BASE_WIDTH);
    ROS_INFO("Automower::WHEEL_PULSES_PER_TURN = %d", WHEEL_PULSES_PER_TURN);
    ROS_INFO("Automower::WHEEL_METER_PER_TICK = %f", WHEEL_METER_PER_TICK);
/*
    // Motortypes:
    // - 1093 - A330 old motors
    // - 349  - A330/320 UltraSilent Motors
    // - 1192 - A320

    // UltraSilent motors are inverted...
    if ((WHEEL_PULSES_PER_TURN == 349) || (WHEEL_PULSES_PER_TURN == 1192))
    {
        invertedMotors = true;
    }
*/

    // PID parameters set for 50z 
    // x factor Adjust to equal regulation as for 50 Hz 
    double x = 50.0/updateRate;  
    
    leftWheelPid.Init(50.0*x, 10.0*x, 1.0*x);
    rightWheelPid.Init(50.0*x, 10.0*x, 1.0*x);

    lastComtestWheelMotorPower = 15;

	char msg1[100];
	snprintf(msg1, sizeof(msg1), "MowerApp.SetMode(modeOfOperation:%d)",IMOWERAPP_MODE_AUTO);

	if (!sendMessage(msg1, sizeof(msg1), result))
	{
		ROS_ERROR("Automower::Failed setting Demo Mode.");
		requestedMode = mode; 
		cuttingDiscOn = lastCuttingDiscOn;
		return false;   
	}


    return true;
}

bool AutomowerSafe::getRealTimeData()
{


    hcp_tResult result;
    const char* msg = "RealTimeData.GetWheelMotorData()";
    if (!sendMessage(msg, sizeof(msg), result))
    {
        return false;   
    }

    /* This value is purely positive...
    int16_t tmpValue = result.parameters[1].value.i16;
    current_lv = ((double)tmpValue) / 1000.0;

    tmpValue = result.parameters[4].value.i16;
    current_rv = ((double)tmpValue) / 1000.0;

    std::cout << "SpeedLeft: " << current_lv << std::endl;
    std::cout << "SpeedRight: " << current_rv << std::endl;
*/

    wheelCurrent.left = result.parameters[2].value.i16;
    wheelCurrent.right = result.parameters[5].value.i16;

    // hcp_tResult result;
    
    const char* msgLv = "Wheels.GetSpeed(index:1)";
    if (!sendMessage(msgLv, sizeof(msgLv), result))
    {
        return false;   
    }
 

    int16_t tmpValue = result.parameters[0].value.i16;
    current_lv = ((double)tmpValue) / 1000.0;

    const char* msgRv = "Wheels.GetSpeed(index:0)";
    if (!sendMessage(msgRv, sizeof(msgRv), result))
    {
        return false;   
    }

    tmpValue = -result.parameters[0].value.i16;
    current_rv = ((double)tmpValue) / 1000.0;

    //std::cout << "SpeedLeft: " << current_lv << std::endl;
    //std::cout << "SpeedRight: " << current_rv << std::endl;


    
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
        //std::cout << "LeftPulses: " << leftPulses << std::endl;
    }

    const char* rightCounterMsg = "Wheels.GetRotationCounter(index:0)";
    if (!sendMessage(rightCounterMsg, sizeof(rightCounterMsg), result))
    {
        return false;   
    }
    if (result.parameterCount > 0)
    {
        rightPulses = -result.parameters[0].value.i32;
        //std::cout << "RightPulses: " << rightPulses << std::endl;
    }

    return true;
}

std::string AutomowerSafe::resultToString(hcp_tResult result)
{
    int i;
    std::ostringstream resultTmp;

    std::string resultStr = "";
    std::string typeName;
    double value;
    if (result.parameterCount < 0 || result.parameterCount > 100)
    {
        std::cout << "invalid number of parameters: " << result.parameterCount << std::cout;
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

bool AutomowerSafe::getSensorData()
{
    hcp_tResult result;

    //
    // SensorStatus
    //
    sensorCheckCounter++;
    if ((sensorCheckCounter % 20) == 0)
    {
        sensorCheckCounter = 0;

        sensorStatus.sensorStatus = 0;

        const char* collisionMsg = "Collision.GetStatus()";
        if (!sendMessage(collisionMsg, sizeof(collisionMsg), result))
        {
            return false;   
        }
        
        if (result.parameterCount == 3)
        {
            //std::cout << "SensorStatus: Check COL: " << (int)result.parameters[0].value.b << std::endl;
            if (result.parameters[0].value.b != 0)
            {
                sensorStatus.sensorStatus |= HVA_SS_COLLISION;
                //std::cout << "SensorStatus: COL" << std::endl;
            }
        }

        const char* chargingMsg = "Charger.IsChargingEnabled()";
        if (!sendMessage(chargingMsg, sizeof(chargingMsg), result))
        {
            return false;   
        }
        if (result.parameterCount == 1)
        {
            //std::cout << "SensorStatus: Check CS: " << (int)result.parameters[0].value.b << std::endl;
            // 0 - Means in CS (this is an ENUM = 0 in the mower)
            if (result.parameters[0].value.b == 0)
            {
                sensorStatus.sensorStatus |= HVA_SS_IN_CS;
                //std::cout << "SensorStatus: IN_CS" << std::endl;
            }
        }

        const char* liftMsg = "LiftSensor.IsActivated()";
        if (!sendMessage(liftMsg, sizeof(liftMsg), result))
        {
            return false;   
        }
        if (result.parameterCount == 1)
        {
            //std::cout << "SensorStatus: Check LIFT: " << (int)result.parameters[0].value.b << std::endl;
            // 1 - Lifted,
            if (result.parameters[0].value.b == 1)
            {
                sensorStatus.sensorStatus |= HVA_SS_LIFTED;
                //std::cout << "SensorStatus: LIFT" << std::endl;
            }
        }
        
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

		if (mode == MODE_PARK)
		{
			sensorStatus.sensorStatus |= HVA_SS_PARKED;
		}


    }



    //
    // State and Mode check
    //

	stateCheckCounter++;
	if (stateCheckCounter >= 10)
	{
		stateCheckCounter = 0;
	
	
		const char* msg1 = "MowerApp.GetMode()";
		if (!sendMessage(msg1, sizeof(msg1), result))
		{
			return false;   
		}
		
		int AM_mode = result.parameters[0].value.u8;
		
		char modeStr[100];
		char stateStr[100];

		switch (AM_mode)
		{
		  case 0:
			strcpy(modeStr, "Auto  ");
			break;
		  case 1:
			strcpy(modeStr, "Manual");
			break;
		  case 2:
			strcpy(modeStr, "Home  ");
			break;
		  case 3:
			strcpy(modeStr, "Demo  ");
			break;
		  
		  default:
			strcpy(modeStr, "Unknown");
		 } 



		const char* msg2 = "MowerApp.GetState()";
		if (!sendMessage(msg2, sizeof(msg2), result))
		{
			return false;   
		}
		int state = result.parameters[0].value.u8;
		switch (state)
		{
		  case 0:
			strcpy(stateStr,"Off  ");
			sensorStatus.operationalMode = AM_OP_MODE_OFFLINE;
			break;
		  case 1:
			sensorStatus.operationalMode = AM_OP_MODE_OFFLINE;
			strcpy(stateStr,"Wait for safety pin");
			mode = MODE_STOPPED;
			break;
		  case 2:
			sensorStatus.operationalMode = AM_OP_MODE_OFFLINE;
			strcpy(stateStr,"Stopped  ");
			mode = MODE_STOPPED;
			break;
		  case 3:
			sensorStatus.operationalMode = AM_OP_MODE_OFFLINE;
			strcpy(stateStr,"Fatal error  ");
			mode = MODE_STOPPED;
			break;
		  case 4:
			sensorStatus.operationalMode = AM_OP_MODE_OFFLINE;
			strcpy(stateStr,"Pending start  ");
			mode = MODE_STOPPED;
			break;
		  case 5:
			sensorStatus.operationalMode = AM_OP_MODE_CONNECTED_MANUAL;
			strcpy(stateStr,"Paused  ");
			mode = MODE_MANUAL;
			break;
		  case 6:
			strcpy(stateStr,"In operation  ");
			sensorStatus.operationalMode = AM_OP_MODE_CONNECTED_RANDOM;
			if (AM_mode == 2)
			{
				mode = MODE_PARK;
			}
			else
			{
				mode = MODE_RANDOM;
			}
			break;
		  case 7:
			strcpy(stateStr,"Restricted  ");
			sensorStatus.operationalMode = AM_OP_MODE_CONNECTED_RANDOM;
			if (AM_mode == 2)
			{
				mode = MODE_PARK;
			}
			else
			{
				mode = MODE_RANDOM;
			}
			break;
		  case 8:
			sensorStatus.operationalMode = AM_OP_MODE_CONNECTED_RANDOM;
			strcpy(stateStr,"Error  ");
			if (AM_mode == 2)
			{
				mode = MODE_PARK;
			}
			else
			{
				mode = MODE_RANDOM;
			}
			break;
		  
		  default:
			strcpy(stateStr,"Unknown");
		 } 
	}

//	std::cout << "AMMode: " << modeStr << "State: " << stateStr << "  om: " << sensorStatus.operationalMode << " rm: " << requestedMode <<  " mode = " << mode << " \r" << std::flush;

	
	


    //
    // Battery check
    //
    batteryCheckCounter++;
    if ((batteryCheckCounter % 50) == 0)
    {
        batteryCheckCounter = 0;



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

        // Send Keep alive message to prevent automower to go to sleep mode
        const char* msgK = "CurrentStatus.GetStatusKeepAlive()";
    	if (!sendMessage(msgK, sizeof(msgK), result))
    	{
    		return false;   	
    	}

    }

    //
    // LoopSensor
    //
    loopSensorCheckCounter++;
    if ((loopSensorCheckCounter % 20) == 0)
    {
        loopSensorCheckCounter = 0;


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

    }


    //
    // STOP button
    //

    const char* userStopMsg = "StopButton.IsActivated()";
    if (!sendMessage(userStopMsg, sizeof(userStopMsg), result))
    {
        return false;   
    }
    if (result.parameterCount == 1)
    {
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
    }

    return true;
}

void AutomowerSafe::stopWheels()
{
	
	// Clear the PIDs and power
	leftWheelPid.Restart();
	rightWheelPid.Restart();

	power_l = 0;
	power_r = 0;

	wheelPower.left = power_l;
	wheelPower.right = power_r;

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
    if ((userStop) || (((requestedMode == MODE_RANDOM)  || (requestedMode == MODE_PARK)) && (mode == MODE_MANUAL)))
    {
		stopWheels();
        return;
    }
    
    if (mode != MODE_MANUAL)
    {
		return;
	}
	

    power_l = leftWheelPid.Update(current_lv, wanted_lv);
    power_r = rightWheelPid.Update(current_rv, wanted_rv);

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

    wheelPower.left = power_l;
    wheelPower.right = power_r;

    // Send it out...
    hcp_tResult result;
    char powerMsg[100];
    snprintf(powerMsg, sizeof(powerMsg), "HardwareControl.WheelMotorsPower(leftWheelMotorPower:%d, rightWheelMotorPower:%d)", power_l, power_r);
    // std::cout << "Send: " << powerMsg << std::endl;
    if (!sendMessage(powerMsg, sizeof(powerMsg), result))
    {
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

    }

    if (serialPortState == AM_SP_STATE_ONLINE)
    {
        ROS_INFO("Automower::Serial port connected!");
        serialPortState = AM_SP_STATE_CONNECTED;


    }


	if (!active)
	{
		return true;
	}

    if (serialComTest)
    {
    	doSerialComTest();
    	return true;
    }


    ros::Time current_time = ros::Time::now();

	if (serialPortState == AM_SP_STATE_CONNECTED)
    {
    	getRealTimeData();
        getSensorData();
        regulateVelocity();
    }



    
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
        ROS_WARN("Automower::Strange distance? => ld = %f, rd = %f", leftDist, rightDist);
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

    // Set this into the pose
    robot_pose.pose.position.x = xpos;
    robot_pose.pose.position.y = ypos;

    tf::Quaternion qyaw = tf::createQuaternionFromYaw(yaw);
    robot_pose.pose.orientation.x = qyaw.x();
    robot_pose.pose.orientation.y = qyaw.y();
    robot_pose.pose.orientation.z = qyaw.z();
    robot_pose.pose.orientation.w = qyaw.w();

    // Publish the pose
    pose_pub.publish(robot_pose);

    // Calculate the TF from the pose...
    tf::Transform transform;
    transform.setOrigin(
        tf::Vector3(robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z));
    tf::Quaternion q;
    tf::quaternionMsgToTF(robot_pose.pose.orientation, q);
    transform.setRotation(q);


    	
    // Send the TF
    if (publishTf)
    {
        br.sendTransform(tf::StampedTransform(transform, current_time, "odom", "base_link"));
    }

    // Odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

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

    // Publish the loop
    loop.header.stamp = current_time;
    loop_pub.publish(loop);

    // Publish the sensorStatus
    if (serialPortState != AM_SP_STATE_CONNECTED)
    {
        // We are offline (i.e. standby?)
        sensorStatus.operationalMode = AM_OP_MODE_OFFLINE;
    }

    sensorStatus.header.stamp = current_time;
    sensorStatus.header.frame_id = "odom";
    sensorStatus_pub.publish(sensorStatus);

    // Publish wheel encoders
    encoder.header.stamp = current_time;
    encoder.lwheel = leftDist;
    encoder.rwheel = rightDist;
    encoder.lwheelAccum = leftPulses;
    encoder.rwheelAccum = rightPulses;
    encoder.lticks = leftTicks;
    encoder.rticks = rightTicks;

    // std::cout << "LeftDist: " << leftDist << " RightDist: " << rightDist;
    // std::cout << " LeftAccum: " << (int)leftPulses << " RightAccum: " << (int)rightPulses << std::endl;

    encoder_pub.publish(encoder);

    // Publish wheel current (updated when read from mower)
    wheelCurrent.header.stamp = current_time;
    wheelCurrent.header.frame_id = "odom";
    current_pub.publish(wheelCurrent);

    wheelPower.header.stamp = current_time;
    wheelPower.header.frame_id = "odom";
    wheelPower_pub.publish(wheelPower);



	char msgA[100];
    hcp_tResult result;

	switch (mode)
	{
		case MODE_MANUAL:
			switch (requestedMode)
			{
				case MODE_MANUAL:
					requestedMode = MODE_UNDEFINED;   // Just come here once after entry.

					if (cuttingDiscOn)
					{
						const char* msg = "BladeMotor.On()";
						if (!sendMessage(msg, sizeof(msg), result))
						{
							return false;   	
						}
						const char* msg1 = "BladeMotor.Run()";
						if (!sendMessage(msg1, sizeof(msg1), result))
						{
							return false;   	
						}
							
					}
					
					
					// Do nothing
					break;
				case MODE_PARK:
					{
						stopWheels();
						
						snprintf(msgA, sizeof(msgA), "MowerApp.SetMode(modeOfOperation:%d)",IMOWERAPP_MODE_HOME);
						if (!sendMessage(msgA, sizeof(msgA), result))
						{
							return false;   	
						}
						const char* msg = "MowerApp.StartTrigger()";
						if (!sendMessage(msg, sizeof(msg), result))
						{
							return false;   	
						}
					}
					break;
				case MODE_RANDOM:
					{
						stopWheels();
						const char* msg = "MowerApp.StartTrigger()";
						if (!sendMessage(msg, sizeof(msg), result))
						{
							return false;   	
						}
					}
					break;
				case MODE_STOPPED:
				case MODE_UNDEFINED:
					// Do nothing
					break;
				default:
					ROS_ERROR("Automower_Safe - Statemachine Error");
			}
			break;
		case MODE_PARK:
			switch (requestedMode)
			{
				case MODE_MANUAL:
					{
						snprintf(msgA, sizeof(msgA), "MowerApp.SetMode(modeOfOperation:%d)",IMOWERAPP_MODE_AUTO);

						if (!sendMessage(msgA, sizeof(msgA), result))
						{
							return false;   	
						}
						const char* msg = "MowerApp.Pause()";

						if (!sendMessage(msg, sizeof(msg), result))
						{
							return false;   	
						}
					}
					break;
				case MODE_PARK:
					// Do nothing
					requestedMode = MODE_UNDEFINED;   // Just come here after entry.
					break;
				case MODE_RANDOM:
					{
						snprintf(msgA, sizeof(msgA), "MowerApp.SetMode(modeOfOperation:%d)",IMOWERAPP_MODE_AUTO);
						if (!sendMessage(msgA, sizeof(msgA), result))
						{
							return false;   	
						}
					}

					break;
				case MODE_STOPPED:
				case MODE_UNDEFINED:
					// Do nothing
					break;
				default:
					ROS_ERROR("Automower_Safe - Statemachine Error");
			}
			break;
		case MODE_RANDOM:
			

			

			switch (requestedMode)
			{
				case MODE_MANUAL:
					{
						const char* msg = "MowerApp.Pause()";
						if (!sendMessage(msg, sizeof(msg), result))
						{
							return false;   	
						}
					}
					
					break;
				case MODE_PARK:
					{
						snprintf(msgA, sizeof(msgA), "MowerApp.SetMode(modeOfOperation:%d)",IMOWERAPP_MODE_HOME);
						if (!sendMessage(msgA, sizeof(msgA), result))
						{
							return false;   	
						}
					}
					break;
				case MODE_RANDOM:
					std::cout << "C3.";
					sendStopCuttingDiscCounter = 0;
					requestedMode = MODE_UNDEFINED;   // Just come here once after entry.
					break;
			 	case MODE_STOPPED:
				case MODE_UNDEFINED:
					break;
				default:
					ROS_ERROR("Automower_Safe - Statemachine Error");
			}

			if (!cuttingDiscOn)
			// Automower automatically puts on cutting disc after a while , stop it explicitly to prevent this.
			{
				sendStopCuttingDiscCounter++;
				if (sendStopCuttingDiscCounter > 40)
				{
					sendStopCuttingDiscCounter = 0;
					const char* msg = "BladeMotor.Brake()";
					if (!sendMessage(msg, sizeof(msg), result))
					{
						return false;   	
					}
				}
			}

			break;
		case MODE_UNDEFINED:
		case MODE_STOPPED:
			switch (requestedMode)
			{
				case MODE_MANUAL:
					ROS_INFO("Automower_Safe - Mower not started");
					break;
				case MODE_PARK:
					ROS_INFO("Automower_Safe - Mower not started");
					break;
				case MODE_RANDOM:
					ROS_INFO("Automower_Safe - Mower not started");
					break;
				case MODE_STOPPED:
				case MODE_UNDEFINED:
					// Do nothing
					break;
				default:
					ROS_ERROR("Automower_Safe - Statemachine Error");
			}
			break;



		default:
			ROS_ERROR("Automower_Safe - Statemachine Error");

	}


	if (cuttingDiscOn != lastCuttingDiscOn)
	{
		if (cuttingDiscOn)
		{
			const char* msg = "BladeMotor.On()";
			if (!sendMessage(msg, sizeof(msg), result))
			{
				return false;   	
			}
			const char* msg1 = "BladeMotor.Run()";
			if (!sendMessage(msg1, sizeof(msg1), result))
			{
				return false;   	
			}
			
				
		}
		else
		{
			const char* msg = "BladeMotor.Brake()";
			if (!sendMessage(msg, sizeof(msg), result))
			{
				return false;   	
			}
		}

		lastCuttingDiscOn = cuttingDiscOn;
	}
	
  
	if (requestedLoopOn != loopOn)
	{
        char msg[100];
		snprintf(msg, sizeof(msg), "SystemSettings.SetLoopDetection(loopDetection:%d)", requestedLoopOn);		
        if (!sendMessage(msg, sizeof(msg), result))
        {
            ROS_ERROR("Automower::Failed setting LoopDetection on/off");
            cuttingHeight = lastCuttingHeight;
            return false;   
        }
        loopOn = requestedLoopOn;
	}
	

    // Cutting height
    if (cuttingHeight != lastCuttingHeight)
    {
        hcp_tResult result;

        ROS_INFO("Automower::set new cutting height= %d mm", cuttingHeight);

        char msg[100];
        snprintf(msg, sizeof(msg), "HeightMotor.SetHeight(height:%d)", cuttingHeight);
        
        if (!sendMessage(msg, sizeof(msg), result))
        {
            ROS_ERROR("Automower::Failed setting cutting height.");
            cuttingHeight = lastCuttingHeight;
            return false;   
        }

        if (result.parameterCount == 1)
        {
            int retVal = result.parameters[0].value.u8;
            if (retVal == 0)
            {
                lastCuttingHeight = cuttingHeight;
            }
            else
            {
                cuttingHeight = lastCuttingHeight;
                ROS_INFO("Automower: Unable to set cutting height ... Not started?");
            }
        }
    }

    return true;
}
}
