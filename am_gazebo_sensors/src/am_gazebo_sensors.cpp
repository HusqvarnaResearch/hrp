#include <map>
#include <string>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/SensorTypes.hh>


#include <tf/tf.h>

#include <am_gazebo_sensors/am_gazebo_sensors.h>

#include <am_driver/SensorStatus.h>

namespace gazebo
{
    
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosSensors)


#define AM_OP_MODE_CONNECTED_SIMULATOR (0x0003)


////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosSensors::GazeboRosSensors() : SensorPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosSensors::~GazeboRosSensors()
{
    rosnode->shutdown();
    callbackQueueThread.join();

    delete rosnode;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosSensors::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
{
    userStopped = false;
#if GAZEBO_MAJOR_VERSION > 2
    parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(parent);
#else 
    parentSensor = boost::dynamic_pointer_cast<sensors::ContactSensor>(parent);
#endif   
    if (!parentSensor)
    {
        ROS_ERROR("Contact sensor parent is not of type ContactSensor");
        return;
    }

    bumperTopicName = "sensor_status";
    if (sdf->GetElement("bumperTopicName"))
    {
        bumperTopicName = sdf->GetElement("bumperTopicName")->Get<std::string>();
    }

    loopTopicName = "loop";
    if (sdf->GetElement("loopTopicName"))
    {
        loopTopicName = sdf->GetElement("loopTopicName")->Get<std::string>();
    }

    loopOutsideSensors = 0xF;
    if (sdf->GetElement("loopOutsideSensors"))
    {
        loopOutsideSensors = sdf->GetElement("loopOutsideSensors")->Get<unsigned int>();
    }
    // ROS_INFO("[Sensors] loopOutsideSensors = 0x%X", loopOutsideSensors);

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    // The ros node
    rosnode = new ros::NodeHandle();

    // Publisher
    contactPub = rosnode->advertise<am_driver::SensorStatus>(std::string(bumperTopicName), 1);

    // Subscribe to loop in order to be "the sensor"
    loopSub = rosnode->subscribe(loopTopicName.c_str(), 1, &GazeboRosSensors::LoopCallback, this);

    // Subscribe to Simulator commands
    simCmdSub = rosnode->subscribe("sim_cmd", 1, &GazeboRosSensors::simCmdCallback, this);

    // Initialize
    // start custom queue for contact bumper
    callbackQueueThread = boost::thread(boost::bind(&GazeboRosSensors::ContactQueueThread, this));

    // Listen to the update event. This event is broadcast every simulation iteration.
    updateConnection = parentSensor->ConnectUpdated(boost::bind(&GazeboRosSensors::OnContact, this));

    // Make sure the parent sensor is active.
    parentSensor->SetActive(true);
}


void GazeboRosSensors::simCmdCallback(const std_msgs::UInt16::ConstPtr& msg)
{
    // User Stopped
    if (msg->data == 0x01)
    {
         userStopped = !userStopped;
         ROS_INFO("GazeboRosSensors::userStopped = %d", userStopped); 
    }
}
////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosSensors::OnContact()
{
    if (contactPub.getNumSubscribers() <= 0)
    {
        return;
    }

    am_driver::SensorStatus status;
    status.sensorStatus = 0x0;
    status.operationalMode = AM_OP_MODE_CONNECTED_SIMULATOR;

    // Collect the loop information and send it out
    if ((loopOutsideSensors & 0x01) && (loop.frontCenter < 0))
    {
        status.sensorStatus |= 0x02;
    }
    if ((loopOutsideSensors & 0x02) && (loop.frontRight < 0))
    {
        status.sensorStatus |= 0x02;
    }
    if ((loopOutsideSensors & 0x04) && (loop.rearLeft < 0))
    {
        status.sensorStatus |= 0x02;
    }
    if ((loopOutsideSensors & 0x08) && (loop.rearRight < 0))
    {
        status.sensorStatus |= 0x02;
    }
    
    if (userStopped)
    {
        
        status.sensorStatus |= 0x80;
        
    }
    // Check if we have contact...
    msgs::Contacts contacts;
#if GAZEBO_MAJOR_VERSION > 2
    contacts = parentSensor->Contacts();
#else
    contacts = parentSensor->GetContacts();
#endif

    if (contacts.contact_size() > 0)
    {
        // Indicate a collision
        status.sensorStatus |= 0x04;

        for (int i = 0; i < contacts.contact_size(); i++)
        {
            std::string target = contacts.contact(i).collision2();
            

            std::string cs_name = "in_cs";
            if (target.find(cs_name) != std::string::npos)
            {
                for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
                {
                    /*std::cout << j << "  Position:"
                        << contacts.contact(i).position(j).x() << " "
                        << contacts.contact(i).position(j).y() << " "
                        << contacts.contact(i).position(j).z() << "\n";
                    std::cout << "   Normal:"
                        << contacts.contact(i).normal(j).x() << " "
                        << contacts.contact(i).normal(j).y() << " "
                        << contacts.contact(i).normal(j).z() << "\n";*/
                    //if (contacts.contact(i).normal(j).x() < 0.2 && (contacts.contact(i).normal(j).y() > 0.8 || contacts.contact(i).normal(j).z() > 0.8))
                    {
                        // std::cout << "------- IN CHARGING STATION ----------" << std::endl;
                        status.sensorStatus |= 0x40;
                        // status.sensorStatus |= 0x10;

                        // Remove collision if we are in CS
                        status.sensorStatus &= ~0x04;
                    }
                    /*else
                    {
                        std::cout << j << "  Position:"
                        << contacts.contact(i).position(j).x() << " "
                        << contacts.contact(i).position(j).y() << " "
                        << contacts.contact(i).position(j).z() << "\n";
                        std::cout << "   Normal:"
                        << contacts.contact(i).normal(j).x() << " "
                        << contacts.contact(i).normal(j).y() << " "
                        << contacts.contact(i).normal(j).z() << "\n";
   
                    }*/
                }
                
            }
            /*
            std::cout << "Collision between[" << contacts.contact(i).collision1()
  << "] and [" << contacts.contact(i).collision2() << "]\n";
*/
        }
    }

    contactPub.publish(status);
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosSensors::ContactQueueThread()
{
    static const double timeout = 0.01;

    while (rosnode->ok())
    {
        contactQueue.callAvailable(ros::WallDuration(timeout));
    }
}

////////////////////////////////////////////////////////////////////////////////
// Collect the loop sensor (from gazebo wire plugin)
void GazeboRosSensors::LoopCallback(const am_driver::Loop::ConstPtr& msg)
{
    loop = *msg;
}
}
