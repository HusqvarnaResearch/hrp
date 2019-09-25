#include <am_gazebo_wheelencoder/am_gazebo_wheelencoder.h>
#include <ros/ros.h>

#include <boost/lexical_cast.hpp>

#include <am_driver/WheelEncoder.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelEncoder);


#define RADIANS_PER_TICK		(M_PI*2.0/1093.0)
#define WHEEL_METER_PER_TICK	(0.000704)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosWheelEncoder::GazeboRosWheelEncoder()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosWheelEncoder::~GazeboRosWheelEncoder()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosWheelEncoder::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
	// Make sure the ROS node for Gazebo has already been initalized
	if (!ros::isInitialized())
	{
		ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. Load the Gazebo system plugin 'libam_gazebo_wheelencoder.so' in the gazebo_ros package)");
		return;
	}

	ROS_INFO("WheelEncoder: am_gazebo_wheelencoder loaded...");
  
	parent = _parent;
	world = _parent->GetWorld();
	
	
	gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "WheelEncoder" ) );
	// Make sure the ROS node for Gazebo has already been initialized
	gazebo_ros_->isInitialized();

	// Publisher
	encoderPub = gazebo_ros_->node()->advertise<am_driver::WheelEncoder>("wheel_encoder", 100);

	// Get the links/joints required...
	
	std::string name = "back_left_wheel";
	leftWheelLink = _parent->GetLink(name);
	
	if (!leftWheelLink)
	{
		ROS_FATAL("WheelEncoder plugin error: link %s does not exist\n", name.c_str());
		return;
	}
	name = "back_left_wheel_joint";
	leftWheelJoint = _parent->GetJoint(name);
	
	if (!leftWheelJoint)
	{
		ROS_FATAL("WheelEncoder plugin error: joint %s does not exist\n", name.c_str());
		return;
	}


	name = "back_right_wheel";
	rightWheelLink = _parent->GetLink(name);
	
	if (!rightWheelLink)
	{
		ROS_FATAL("WheelEncoder plugin error: link %s does not exist\n", name.c_str());
		return;
	}
	name = "back_right_wheel_joint";
	rightWheelJoint = _parent->GetJoint(name);
	
	if (!rightWheelJoint)
	{
		ROS_FATAL("WheelEncoder plugin error: joint %s does not exist\n", name.c_str());
		return;
	}

#if GAZEBO_MAJOR_VERSION >= 8
   lastLeftAngle = leftWheelJoint->Position ( 0 );
	lastRightAngle = rightWheelJoint->Position ( 0 );
#else
   lastLeftAngle = leftWheelJoint->GetAngle(0).Radian();
	lastRightAngle = rightWheelJoint->GetAngle(0).Radian();
#endif

	

 	// listen to the update event (broadcast every simulation iteration)
	//updateConnection = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosWheelEncoder::Update, this ) );

	// connect Update function
	updateTimer.setUpdateRate(50.0);
	updateTimer.Load(world, _sdf);
	updateConnection = updateTimer.Connect(boost::bind(&GazeboRosWheelEncoder::Update, this));	


}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosWheelEncoder::Update()
{

	// Get the actual angle of the Joints
	
	// LEFT
#if GAZEBO_MAJOR_VERSION >= 8
   double leftAngle = leftWheelJoint->Position ( 0 );
#else
   double leftAngle = leftWheelJoint->GetAngle(0).Radian();
#endif

	double dLeftAngle = leftAngle - lastLeftAngle;
	lastLeftAngle = leftAngle;
	//std::cout << "LeftAngle: " <<  leftAngle*180/M_PI << std::endl;
	
	double leftPulses = dLeftAngle / RADIANS_PER_TICK;
	double leftDist = leftPulses * WHEEL_METER_PER_TICK;

	// RIGHT
#if GAZEBO_MAJOR_VERSION >= 8
   double rightAngle = rightWheelJoint->Position ( 0 );
#else
   double rightAngle = rightWheelJoint->GetAngle(0).Radian();
#endif
	double dRightAngle = rightAngle - lastRightAngle;
	lastRightAngle = rightAngle;
	//std::cout << "RightAngle: " <<  rightAngle*180/M_PI << std::endl;
	
	double rightPulses = dRightAngle / RADIANS_PER_TICK;
	double rightDist = rightPulses * WHEEL_METER_PER_TICK;

	// ACCUM 
	double leftAccum = -leftAngle/RADIANS_PER_TICK;
	double rightAccum = rightAngle/RADIANS_PER_TICK;

	// PUBLISH
	//std::cout << "LeftDist: " << leftDist << " RightDist: " << rightDist;
	//std::cout << " LeftAccum: " << leftAccum << " RightAccum: " << rightAccum << std::endl;

	// Limit if small values...
	if (fabs(leftDist) < 0.0001)
	{
		leftDist = 0.0;
	}
	if (fabs(rightDist) < 0.0001)
	{
		rightDist = 0.0;
	}


	am_driver::WheelEncoder encoder;
		
	encoder.header.frame_id = "wheel_encoder";
	encoder.header.stamp = ros::Time::now();
	encoder.lwheel = leftDist;
	encoder.rwheel = rightDist;
	encoder.lwheelAccum = leftAccum;
	encoder.rwheelAccum = rightAccum;
	
	encoderPub.publish(encoder);
}

}
