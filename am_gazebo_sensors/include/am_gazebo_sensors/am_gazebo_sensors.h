#ifndef GAZEBO_ROS_SENSORS_H
#define GAZEBO_ROS_SENSORS_H

#include <string>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <sys/time.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <std_msgs/String.h>

#include <gazebo/sensors/sensors.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/common/Plugin.hh>
#include <std_msgs/UInt16.h>
#include <am_driver/Loop.h>

namespace gazebo
{
class GazeboRosSensors : public SensorPlugin
{
public:
    GazeboRosSensors();
    ~GazeboRosSensors();

    void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf);

private:
    void OnContact();
    void LoopCallback(const am_driver::Loop::ConstPtr& msg);
    void simCmdCallback(const std_msgs::UInt16::ConstPtr& msg);
    // ROS STUFF
    ros::NodeHandle* rosnode;
    ros::Publisher contactPub;
    ros::Subscriber loopSub;
    ros::Subscriber simCmdSub;
    // GAZEBO STUFF
    sensors::ContactSensorPtr parentSensor;
    ros::CallbackQueue contactQueue;
    void ContactQueueThread();
    boost::thread callbackQueueThread;
    event::ConnectionPtr updateConnection;

    // SENSOR STUFF
    std::string bumperTopicName;
    std::string loopTopicName;

    unsigned int loopOutsideSensors;
    bool userStopped;
    am_driver::Loop loop;
};
}

#endif
