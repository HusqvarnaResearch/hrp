/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#include "am_driver_legacy/automower_legacy.h"
#include "am_driver_legacy/amproto_legacy.h"
#include <tf/transform_datatypes.h>

#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>

#define DEG2RAD(DEG) ((DEG) * ((M_PI) / (180.0)))

#define AUTMOWER_WHEEL_BASE_WIDTH (0.4645)

#define MODE_MANUAL (0x0000)
#define MODE_RANDOM (0x0001)

// OPERATIONAL MODES (i.e. published in SensorStatus)
#define AM_OP_MODE_OFFLINE (0x0000)
#define AM_OP_MODE_CONNECTED_MANUAL (0x0001)
#define AM_OP_MODE_CONNECTED_RANDOM (0x0002)

#define AM_CTRL_THRESHOLD (0.05)
#define AM_CTRL_INCREASE (1.00)
#define AM_CTRL_DECREASE (1.00)
#define AM_CTRL_MAX_SPEED (0.50)
#define AM_CTRL_MIN_SPEED (-0.50)

#define AM_SP_STATE_OFFLINE (0)
#define AM_SP_STATE_ONLINE (1)
#define AM_SP_STATE_CONNECTED (2)

#define AM_MAINBOARD_NEEDS_CONFIG (0)
#define AM_MAINBOARD_ROS_IN_CONTROL (1)
#define AM_MAINBOARD_HMB_IN_CONTROL (2)

namespace Husqvarna
{

AutomowerLegacy::AutomowerLegacy(const ros::NodeHandle& nodeh)
{
    // Init attributes
    nh = nodeh;
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

    // Setup some ROS stuff
    cmd_sub = nh.subscribe("cmd_mode", 5, &AutomowerLegacy::modeCallback, this);

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

    encoder_pub = nh.advertise<am_driver::WheelEncoder>("wheel_encoder", 1);
    current_pub = nh.advertise<am_driver::WheelCurrent>("wheel_current", 1);

    imuResetSub = nh.subscribe("imu_reset", 1, &AutomowerLegacy::imuResetCallback, this);

    velocity_sub = nh.subscribe("cmd_vel", 1, &AutomowerLegacy::velocityCallback, this);

    loop_pub = nh.advertise<am_driver::Loop>("loop", 1);
    sensorStatus_pub = nh.advertise<am_driver::SensorStatus>("sensor_status", 1);

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

    tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
    robot_pose.pose.orientation.x = q.x();
    robot_pose.pose.orientation.y = q.y();
    robot_pose.pose.orientation.z = q.z();
    robot_pose.pose.orientation.w = q.w();

    robot_pose.header.frame_id = "base_link";
    robot_pose.header.stamp = ros::Time::now();

    imuOffset = 0.0;

    cuttingDiscOn = false;

    cuttingHeight = 20;
    lastCuttingHeight = 20;

    serialPortState = AM_SP_STATE_OFFLINE;

    reqFinished = false;

    actionResponse = 0;

    reqFrontOutside = false;
    reqRearOutside = false;

    invertedMotors = false;
}

AutomowerLegacy::~AutomowerLegacy()
{
    if (serialFd >= 0)
    {
        close(serialFd);
    }
}

bool AutomowerLegacy::setup()
{
    struct termios term;

    ROS_INFO("AutomowerLegacy::setup()");

    // Open serial port
    serialFd = open(pSerialPort.c_str(), O_RDWR /*| O_NONBLOCK */);
    if (serialFd < 0)
    {
        ROS_ERROR("AutomowerLegacy::Serial port open:");
        return false;
    }

    /*
            if (tcgetattr(serialFd, &term) < 0 )
            {
                    ROS_ERROR("AutomowerLegacy::setup():tcgetattr():");
                    close(serialFd);
                    serialFd = -1;
                    return false;
            }

            //cfmakeraw(&term);
            cfsetispeed(&term, B115200);
            cfsetospeed(&term, B115200);

            term.c_lflag &= ~ICANON;
            term.c_cc[VTIME] = 20;
            term.c_cc[VMIN] = 0;

            if (tcflush(serialFd, TCIOFLUSH ) < 0)
            {
                    ROS_ERROR("AutomowerLegacy::setup():tcflush():");
                    close(serialFd);
                    serialFd = -1;
                    return false;
            }

            if (tcsetattr(serialFd, TCSANOW, &term ) < 0)
            {
                    ROS_ERROR("AutomowerLegacy::setup():tcsetattr():");
                    close(serialFd);
                    serialFd = -1;
                    return false;
            }
    */

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
void AutomowerLegacy::imuResetCallback(const geometry_msgs::Pose::ConstPtr& msg)
{

    ROS_INFO("AutomowerLegacy::imuResetCallback!");
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

void AutomowerLegacy::velocityCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
    last_command_time = ros::WallTime::now();
    lin_vel = (double)vel->linear.x;
    ang_vel = (double)vel->angular.z;

    wanted_lv = lin_vel - ang_vel * AUTMOWER_WHEEL_BASE_WIDTH / 2;
    wanted_rv = lin_vel + ang_vel * AUTMOWER_WHEEL_BASE_WIDTH / 2;

    // ROS_INFO("AutomowerLegacy::cmd_vel: %f m/s  %f rad/s => wanted_lv=%f, wanted_rv=%f", (float)lin_vel, (float)ang_vel,
    // (float)wanted_lv, (float)wanted_rv);
}

void AutomowerLegacy::modeCallback(const std_msgs::UInt16::ConstPtr& msg)
{
    if (msg->data < 0x90)
    {
        // Not for us...
        return;
    }

    if (msg->data == 0x90)
    {
        mode = MODE_MANUAL;
    }
    else if (msg->data == 0x91)
    {
        mode = MODE_RANDOM;
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
        reqFinished = true;
        ROS_INFO("FINISHED requested");
    }
    else if (msg->data == 0x101)
    {
        reqFinished = false;
        ROS_INFO("FINISHED cleared");
    }
    else if (msg->data == 0x102)
    {
        reqFrontOutside = true;
        ROS_INFO("FRONT OUTSIDE geofence requested");
    }
    else if (msg->data == 0x103)
    {
        reqFrontOutside = false;
        ROS_INFO("FRONT OUTSIDE geofence cleared");
    }
    else if (msg->data == 0x104)
    {
        reqRearOutside = true;
        ROS_INFO("REAR OUTSIDE geofence requested");
    }
    else if (msg->data == 0x105)
    {
        reqRearOutside = false;
        ROS_INFO("REAR OUTSIDE geofence cleared");
    }
    else
    {
        // Do nothing...probably not for me...
        return;
    }

    // ROS_INFO("AutomowerLegacy::cmd_mode: %d", msg->data);
}

bool AutomowerLegacy::initAutomowerBoard()
{
    ROS_INFO("AutomowerLegacy::initAutomowerBoard");

    NavigationSetup navSetup;

    unsigned char msg[AM_PROTO_MAX_LENGTH];
    unsigned char resp[AM_PROTO_MAX_LENGTH];

    int messageLength = navSetup.getMessage((unsigned char*)&msg);

    int reclength = sendMessage(msg, messageLength, resp, AM_PROTO_MAX_LENGTH, true);

    if (reclength == 0)
    {
        ROS_ERROR("AutomowerLegacy::Failed to init mower!!!");
        return false;
    }

    // Get some stuff out from the mower...
    WHEEL_SPEED_TIMER_TICK = (resp[15] << 24) + (resp[14] << 16) + (resp[13] << 8) + (resp[12] << 0);
    int dia = (resp[7] << 8) + (resp[6] << 0);
    WHEEL_DIAMETER = (double)(dia) / 1000.0;

    WHEEL_PULSES_PER_TURN = (resp[5] << 8) + (resp[4] << 0);
    WHEEL_METER_PER_TICK = (2.0 * M_PI * WHEEL_DIAMETER / 2.0) / (double)WHEEL_PULSES_PER_TURN;

    ROS_INFO("AutomowerLegacy::WHEEL_SPEED_TIMER_TICK = %d", WHEEL_SPEED_TIMER_TICK);
    ROS_INFO("AutomowerLegacy::WHEEL_DIAMETER = %f", WHEEL_DIAMETER);
    ROS_INFO("AutomowerLegacy::WHEEL_PULSES_PER_TURN = %d", WHEEL_PULSES_PER_TURN);
    ROS_INFO("AutomowerLegacy::WHEEL_METER_PER_TICK = %f", WHEEL_METER_PER_TICK);

    // Motortypes:
    // - 1093 - A330 old motors
    // - 349  - A330/320 UltraSilent Motors
    // - 1192 - A320

    // UltraSilent motors are inverted...
    if ((WHEEL_PULSES_PER_TURN == 349) || (WHEEL_PULSES_PER_TURN == 1192))
    {
        invertedMotors = true;
    }

    return true;
}

int AutomowerLegacy::sendMessage(unsigned char* msg, int len, unsigned char* ansmsg, int maxAnsLength, bool retry)
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
    int cnt = 0;
    cnt = write(serialFd, msg, len);

    if (cnt != len)
    {
        ROS_ERROR("AutomowerLegacy::Could not send on serial port!");
        return -1;
    }

    cnt = 0;
    int res;
    int payloadLength = 0;

    // Clear answer buffer
    memset(ansmsg, 0, maxAnsLength);

    // Keep reading until we find an STX
    while (ansmsg[cnt] != 0x02)
    {
        res = read(serialFd, &ansmsg[cnt], 1);

        if (res <= 0)
        {
            // ROS_WARN("AutomowerLegacy::Failed to get STX...sleeping?");
            serialPortState = AM_SP_STATE_OFFLINE;
            return 0;
        }
    }

    cnt++;

    // Read MESSAGE TYPE
    res = read(serialFd, &ansmsg[cnt], 1);
    // std::cout << "MESSAGE TYPE: " << (int)ansmsg[cnt] << std::endl;
    cnt++;

    // Read LENGTH
    res = read(serialFd, &ansmsg[cnt], 1);
    payloadLength = ansmsg[cnt];
    // std::cout << "PAYLOAD LENGTH: " << (int)payloadLength << std::endl;
    cnt++;

    // Read PAYLOAD
    unsigned char readBytes = 0;
    unsigned max_retries = 100;
    while ((readBytes < payloadLength) && (max_retries > 0))
    {
        int bytes = read(serialFd, &ansmsg[cnt], payloadLength - readBytes);
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
        ROS_WARN("AutomowerLegacy::Failed STX");
        return 0;
    }
    if (ansmsg[cnt - 1] != 0x03)
    {
        // FAILED
        ROS_WARN("AutomowerLegacy::Failed ETX");
        return 0;
    }

    return cnt;
}

int AutomowerLegacy::twosComp(int val, int bits)
{
    // compute the 2's compliment of int value val
    if ((val & (1 << (bits - 1))) != 0)
    {
        val = val - (1 << bits);
    }
    return val;
}

double AutomowerLegacy::regulateVelocity(ros::Duration dt, double wanted_vel, double actual_vel)
{
    double res = actual_vel;

    if (fabs(wanted_vel - actual_vel) < AM_CTRL_THRESHOLD)
    {
        // We are ok...
        res = wanted_vel;
        // ROS_INFO("Within threshold...");
    }
    else if (wanted_vel > actual_vel)
    {
        // We are too slow...increase!
        res = actual_vel + AM_CTRL_INCREASE * dt.toSec();

        if (res > wanted_vel)
        {
            res = wanted_vel;
        }

        if (res > AM_CTRL_MAX_SPEED)
        {
            res = AM_CTRL_MAX_SPEED;
        }
        // ROS_INFO("Increase");
    }
    else
    {
        // We are too fast...decrease!
        res = actual_vel - AM_CTRL_DECREASE * dt.toSec();

        if (res < wanted_vel)
        {
            res = wanted_vel;
        }

        if (res < AM_CTRL_MIN_SPEED)
        {
            res = AM_CTRL_MIN_SPEED;
        }
        // ROS_INFO("Decrease");
    }

    return res;
}

void AutomowerLegacy::doAccelerationControl(ros::Duration dt)
{
    if (velocityRegulator == 1)
    {
        // Regulate the speed (left and right)
        current_lv = regulateVelocity(dt, wanted_lv, current_lv);
        current_rv = regulateVelocity(dt, wanted_rv, current_rv);
        // ROS_INFO("current_vel: %f, %f", current_lv, current_rv);

        // Make sure we send 0.0 m/s when close
        if (fabs(current_lv) < 0.01)
        {
            current_lv = 0.0;
        }
        if (fabs(current_rv) < 0.01)
        {
            current_rv = 0.0;
        }
    }
    else
    {
        // Just hit it...
        current_lv = wanted_lv;
        current_rv = wanted_rv;
    }
}

bool AutomowerLegacy::sendNavigation(signed short lv, signed short rv)
{
    // The same message for getting of odometry as setting new speed
    // hence the odo class here will have to also be responsible for
    // setting the speed (calculated by motor class).
    Navigation nav;

    if (mode == MODE_RANDOM)
    {
        nav.releaseControl();
    }
    else
    {
        nav.requestControl();
    }

    nav.setSpeed(lv, rv);

    unsigned short status = 0x00;

    if (cuttingDiscOn)
    {
        status |= 0x20;
    }

    if (reqFinished)
    {
        status |= 0x10;
    }

    if (reqFrontOutside)
    {
        status |= 1 << 14;
    }
    if (reqRearOutside)
    {
        status |= 1 << 13;
    }
    nav.setStatus(status);

    unsigned char msg[AM_PROTO_MAX_LENGTH];
    unsigned char resp[AM_PROTO_MAX_LENGTH];

    int messageLength = nav.getMessage((unsigned char*)&msg);
    int reclength = sendMessage(msg, messageLength, resp, AM_PROTO_MAX_LENGTH, true);
    if (reclength == 0)
    {
        return false;
    }

    // Get the status of the main board
    actionResponse = resp[4];

    // Pulses
    leftPulses = (resp[8] << 24) + (resp[7] << 16) + (resp[6] << 8) + (resp[5] << 0);
    rightPulses = (resp[12] << 24) + (resp[11] << 16) + (resp[10] << 8) + (resp[9] << 0);
    
    if (invertedMotors)
    {
        leftPulses = -leftPulses;
        rightPulses = -rightPulses;
    }

    // First time only...
    if (!automowerInterfaceInited)
    {
        automowerInterfaceInited = true;
        lastLeftPulses = leftPulses;
        lastRightPulses = rightPulses;
    }

    // Status for sensors, e.g. collision, lift, etc...
    sensorStatus.sensorStatus = (resp[14] << 8) + resp[13];

    // 15..16 (max wheel speed, suggested)
    int maxWheelSpeed = (resp[16] << 8) + resp[15];

    // 17..18 (error code) 0 = OK
    int errorCode = (resp[18] << 8) + resp[17];

    leftTicks = (resp[22] << 24) + (resp[21] << 16) + (resp[20] << 8) + (resp[19] << 0);
    rightTicks = (resp[26] << 24) + (resp[25] << 16) + (resp[24] << 8) + (resp[23] << 0);

    // Save the loop data for sending
    // loop.header.stamp = ros::Time::now();
    loop.header.frame_id = "loop_base";

    // 27..28 - Signed, A0 from FrontCenter
    loop.frontCenter = twosComp((resp[28] << 8) + resp[27], 16);
    // 29..30 - Signed, A0 from FrontRight
    loop.frontRight = twosComp((resp[30] << 8) + resp[29], 16);
    // 31..32 - Signed, A0 from RearLeft
    loop.rearLeft = twosComp((resp[32] << 8) + resp[31], 16);
    // 33..34 - Signed, A0 from RearRight
    loop.rearRight = twosComp((resp[34] << 8) + resp[33], 16);

    // Left/Right motor current
    // 35..36 Left
    wheelCurrent.left = twosComp((resp[36] << 8) + resp[35], 16);
    // 37..38 Right
    wheelCurrent.right = twosComp((resp[38] << 8) + resp[37], 16);

    // ROS_INFO("AutomowerLegacy::lv = %d, rv = %d", lv, rv);
    // ROS_INFO("AutomowerLegacy::leftPulses = %d, rightPulses = %d", leftPulses, rightPulses);

    return true;
}

bool AutomowerLegacy::update(ros::Duration dt)
{
    ros::Time current_time = ros::Time::now();

    if (serialPortState == AM_SP_STATE_OFFLINE)
    {
        if (initAutomowerBoard())
        {
            ROS_INFO("AutomowerLegacy::Serial port ONLINE!");
            serialPortState = AM_SP_STATE_ONLINE;
        }
        else
        {
            ROS_WARN("AutomowerLegacy::Failed to contact Mower board - SLEEPING?");
        }
    }

    if (serialPortState == AM_SP_STATE_ONLINE)
    {
        ROS_INFO("AutomowerLegacy::Serial port connected!");
        serialPortState = AM_SP_STATE_CONNECTED;
    }

    doAccelerationControl(dt);

    // Automower wants to have mm/s
    signed short lv_mm = current_lv * 1000;
    signed short rv_mm = current_rv * 1000;

    // Only send messages if we are connected...else we just publish data
    // but indicate in the sensorStatus that we are not online...
    if (serialPortState == AM_SP_STATE_CONNECTED)
    {
        bool res = sendNavigation((signed short)lv_mm, (signed short)rv_mm);
        if (!res)
        {
            ROS_ERROR("AutomowerLegacy::Failed Navigation message => OFFLINE!");
            serialPortState = AM_SP_STATE_OFFLINE;
            return false;
        }
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
        ROS_WARN("AutomowerLegacy::Strange distance? => ld = %f, rd = %f", leftDist, rightDist);
        leftDist = 0;
        rightDist = 0;
    }

    // ROS_INFO("AutomowerLegacy::ld = %f, rd = %f", leftDist, rightDist);

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

    // ROS_INFO("AutomowerLegacy::pos: dt=%f xpos=%f, ypos=%f", (dt.toSec()), (float)xpos, (float)ypos);

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
    sensorStatus.operationalMode = 0;
    if (serialPortState == AM_SP_STATE_CONNECTED)
    {
        // In which ONLINE mode?
        if (actionResponse == AM_MAINBOARD_ROS_IN_CONTROL)
        {
            sensorStatus.operationalMode = AM_OP_MODE_CONNECTED_MANUAL;
        }
        else
        {
            sensorStatus.operationalMode = AM_OP_MODE_CONNECTED_RANDOM;
        }
    }
    else
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

    // Cutting height
    if (cuttingHeight != lastCuttingHeight)
    {
        ROS_INFO("AutomowerLegacy::set new cutting height= %d mm", cuttingHeight);

        unsigned char msg[AM_PROTO_MAX_LENGTH];
        unsigned char resp[AM_PROTO_MAX_LENGTH];
        int messageLength;
        int reclength;

        CuttingHeight height;

        height.setCuttingHeight(cuttingHeight);
        messageLength = height.getMessage((unsigned char*)&msg);
        reclength = sendMessage(msg, messageLength, resp, AM_PROTO_MAX_LENGTH, true);
        if (reclength == 0)
        {
            ROS_ERROR("AutomowerLegacy::Failed setting cutting height.");
        }

        lastCuttingHeight = cuttingHeight;
    }

    return true;
}
}
