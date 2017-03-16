#ifndef GAZEBO_ROS_WIRE_H
#define GAZEBO_ROS_WIRE_H

#include <ros/ros.h>

#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <am_driver/Loop.h>
#include <am_driver/LoopData.h>

namespace gazebo
{
	
class GazeboRosWire : public ModelPlugin
{
public:
	/// \brief Constructor
	GazeboRosWire();

	/// \brief Destructor
	virtual ~GazeboRosWire();

	/// \brief Load the controller
	void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

	/// \brief Update the controller
protected: 
	virtual void Update();

private:
	typedef struct
		{
			double mapWidth;
			double mapHeight;
			unsigned int imgWidth;
			unsigned int imgHeight;
			std::vector<int16_t> magField;
			std::string name;
		} loopData;

	void InitPlugin();
	bool rosStarted;
	
	int GetLoopValue(math::Pose pose,const loopData& loop);




	gazebo::GazeboRosPtr gazebo_ros_;

	// Simulation objects
	physics::WorldPtr theWorld;
	physics::ModelPtr theWire;
	physics::ModelPtr theMower;
	physics::LinkPtr  theGardenLink;
	
	// Used for loop/wire calculations from and Image
	math::Pose gardenPose;
	std::string wireBitmapName;



	//FrKa
	std::vector<loopData> loopVec;


	int16_t GetMagValue(math::Pose pose);
	void ExtractBinData(const std::string& filename,loopData& loop);
	void ExtractImageData(const std::string& filename,loopData& loop);

	// The ROS loop messages
	ros::Publisher loopPub;
	am_driver::Loop loop;
	
	// Internal links to use for simulation
	physics::LinkPtr robotLink;
	physics::LinkPtr frontCenterLink;
	physics::LinkPtr frontRightLink;
	physics::LinkPtr rearRightLink;
	physics::LinkPtr rearLeftLink;
	
	event::ConnectionPtr updateConnection;
};
	
}

#endif
