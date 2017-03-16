#ifndef GAZEBO_ROS_WHEELENCODER_HH
#define GAZEBO_ROS_WHEELENCODER_HH

#include <ros/ros.h>

#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <am_gazebo_wheelencoder/update_timer.h>


namespace gazebo
{
	

class GazeboRosWheelEncoder : public ModelPlugin
{
	/// \brief Constructor
	public: GazeboRosWheelEncoder();

	/// \brief Destructor
	public: virtual ~GazeboRosWheelEncoder();

	/// \brief Load the controller
	public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

	/// \brief Update the controller
protected: 
	virtual void Update();

private:
	gazebo::GazeboRosPtr gazebo_ros_;
	physics::ModelPtr parent;
	physics::WorldPtr world;
	
	ros::Publisher encoderPub;
	
	physics::LinkPtr leftWheelLink;
	physics::LinkPtr rightWheelLink;

	physics::JointPtr leftWheelJoint;
	physics::JointPtr rightWheelJoint;
	
	event::ConnectionPtr updateConnection;
	gazebo::UpdateTimer updateTimer;
	
	double lastLeftAngle;
	double lastRightAngle;
	
};
	
}

#endif
