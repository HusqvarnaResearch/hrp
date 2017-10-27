/*
 * vivepos.h
 *
 */

#ifndef AUTOMOWER_HAL_AM_VIVE_POS_H_
#define AUTOMOWER_HAL_AM_VIVE_POS_H_

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <nav_msgs/Odometry.h>
#include <deque>
#include <mutex>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>


namespace Husqvarna {

class VivePos {
public:
	VivePos(const ros::NodeHandle& nodeh);
	virtual ~VivePos();

	void Run();

private:
    // ROS data
    ros::NodeHandle nh;
    ros::Publisher odom_pub;
		ros::Publisher pose_pub;
    std::string pSerialUsbPort;

		void publish();
		void handleLine(std::string line);

		int publishTf;

		double xPos;
		double yPos;
		double zPos;

		double roll;
		double pitch;
		double yaw;

		tf::TransformBroadcaster br;
		tf::Transform transform;
		geometry_msgs::PoseStamped robot_pose;
};


typedef boost::shared_ptr<VivePos> VivePosPtr;
} /* namespace Husqvarna */

#endif /* AUTOMOWER_HAL_AM_VIVE_POS_H_ */
