#include "am_driver_safe/automower_safe.h"

namespace Husqvarna
{

AutomowerStrict::AutomowerStrict(const ros::NodeHandle& nodeh, decision_making::RosEventQueue* eq) :
    AutomowerSafe(nodeh, eq),
    updateCounter(0)
{
}

bool AutomowerStrict::update(ros::Duration dt)
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

    if (serialPortState == AM_SP_STATE_CONNECTED )
    {
        updateCounter++;
        ros::Time current_time;
        if (wheelSensorFreq != 0.)
        {
            current_time = ros::Time::now();
            // Get data in strict sequence, in combination with a carefully selected call frequency to this function this will ensure that data is
            // retrieved at a stable frequency
            getWheelData();
            ros::Duration wheelDt = ros::Time::now() - current_time;
            ROS_DEBUG_STREAM("wheelDt: " << wheelDt.nsec / 1e6 << "ms");
            motorFeedbackDiffDrive_pub.publish(motorFeedbackDiffDrive);
        }

        if (loopSensorFreq != 0.)
        {
            current_time = ros::Time::now();
            getLoopA0Data();
            ros::Duration loopDt = ros::Time::now() - current_time;
            ROS_DEBUG_STREAM("loopDt: " << loopDt.nsec / 1e6 << "ms");
            loop_pub.publish(loop);
        }

        if (regulatorFreq != 0.)
        {
            current_time = ros::Time::now();
            if (velocityRegulator)
            {
                ROS_DEBUG_STREAM("Sensor status: " << sensorStatus.sensorStatus);
                regulateVelocity();
            }
            else
            {
                setPower();
            }
            ros::Duration regulateDt = ros::Time::now() - current_time;
            ROS_DEBUG_STREAM("regulateDt: " << regulateDt.nsec / 1e6 << "ms");
        }

        // Split misc tasks between update calls to make sure wheel data and loop signal are retrieved in desired frequency.
        if (sensorStatusCheckFreq != 0.)
        {
            if (updateCounter == 1)
            {
                sensorStatus.sensorStatus = 0;
                current_time = ros::Time::now();
                getLoopDetection();

                ros::Duration dt = ros::Time::now() - current_time;
                ROS_DEBUG_STREAM("loopDetectionDt: " << dt.nsec / 1e6 << "ms");
            }
            if (updateCounter == 3)
            {
                current_time = ros::Time::now();
                getStatus();

                ros::Duration dt = ros::Time::now() - current_time;
                ROS_DEBUG_STREAM("getStatusDt: " << dt.nsec / 1e6 << "ms");
            }
            if (updateCounter == 5)
            {
                current_time = ros::Time::now();
                getChargingPowerConnected();

                // Publish the sensorStatus
                if (serialPortState != AM_SP_STATE_CONNECTED)
                {
                    // We are offline (i.e. standby?)
                    sensorStatus.operationalMode = AM_OP_MODE_OFFLINE;
                }

                sensorStatus.header.stamp = current_time;
                sensorStatus.header.frame_id = "odom";
                sensorStatus_pub.publish(sensorStatus);

                ros::Duration dt = ros::Time::now() - current_time;
                ROS_DEBUG_STREAM("chargeConnectedDt: " << dt.nsec / 1e6 << "ms");
            }
        }
        if (batteryCheckFreq != 0. && updateCounter == 6)
        {
            current_time = ros::Time::now();
            getBatteryData();
            ros::Duration dt = ros::Time::now() - current_time;
            ROS_DEBUG_STREAM("getBatteryDataDt: " << dt.nsec / 1e6 << "ms");
        }
        if (updateCounter == 7)
        {
            current_time = ros::Time::now();
            getStatusKeepAlive();
            ros::Duration dt = ros::Time::now() - current_time;
            ROS_DEBUG_STREAM("keepAliveDt: " << dt.nsec / 1e6 << "ms");

            currentStatus.header.stamp = current_time;
            currentStatus.header.frame_id = "odom";
            currentStatus_pub.publish(currentStatus);
        }
        if (stateCheckFreq && updateCounter == 9)
        {
            current_time = ros::Time::now();
            getStateData();
            ros::Duration dt = ros::Time::now() - current_time;
            ROS_DEBUG_STREAM("stateDt: " << dt.nsec / 1e6 << "ms");
        }

        if (updateCounter == 10)
        {
            updateCounter = 0;
        }
    }
    return true;
}

bool AutomowerStrict::getLoopA0Data()
{
    //
    // LoopSensor
    //
    hcp_tResult result;
    ros::Time current_time = ros::Time::now();
    const char* loopAmsg = "LoopSampler.GetLoopSignalMaster(loop:0)";
    if (!sendMessage(loopAmsg, sizeof(loopAmsg), result))
    {
        return false;
    }
    if (result.parameterCount == 1)
    {
        loop.header.stamp = current_time;
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

    return true;
}

}
