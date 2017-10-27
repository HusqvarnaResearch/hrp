/*
 * vivepos.cpp
 *
 */

#include "vivepos.h"
#include <am_vive_pos/timeoutserial.h>
#include <std_msgs/Float32.h>
#include "std_msgs/UInt32.h"
#include <string.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

#include <tf/transform_datatypes.h>

#define DEG2RAD(DEG) ((DEG) * ((M_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * ((180.0) / (M_PI)))
#define NOISE(n) (((rand() % 1000) / 1000.0) * n - n / 2.0)

#define FIX_ANGLES(a)       \
    if (a > M_PI * 2.0)     \
    {                       \
        a = a - M_PI * 2.0; \
    }                       \
    else if (a < 0.0)       \
    {                       \
        a = a + M_PI * 2.0; \
    }


namespace Husqvarna {

VivePos::VivePos(const ros::NodeHandle& nodeh)
{
	  // Init attributes
    nh = nodeh;

    // Parameters
    ros::NodeHandle n_private("~");

    std::string defPort = "/dev/ttyACM0"; //"/dev/pts/22";
    n_private.param("serialUsbPort", pSerialUsbPort, defPort);

    n_private.param("publishTf", publishTf, 1);
    ROS_INFO("Param: publishTf: [%d]", publishTf);

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom_combined",5);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 5);

    // Initialize the intial pose
    robot_pose.pose.position.x = 0.0;
    robot_pose.pose.position.y = 0.0;
    robot_pose.pose.position.z = 0.0;

    xPos = 0.0;
    yPos = 0.0;
    zPos = 0.0;

}

VivePos::~VivePos()
{
	// TODO Auto-generated destructor stub
}

void VivePos::Run()
{
  try
  {
      TimeoutSerial serial(pSerialUsbPort, 115200);
      serial.setTimeout(boost::posix_time::seconds(15));

      while (ros::ok())
      {
          // Read from serial port
          std::string line;

          try
          {
              line = serial.readStringUntil("\n");
              handleLine(line);
          }
          catch (timeout_exception)
          {
              ROS_WARN("No range data???");
          }
      }
  }
  catch (boost::system::system_error& e)
  {
      ROS_ERROR("Range-serial: %s", e.what());
  }

}

void VivePos::handleLine(std::string line)
{
    // Parse data
    if (boost::starts_with(line, "OBJ0"))
    {
       std::vector<std::string> strs;
       boost::split(strs, line, boost::is_any_of(" \t"));
       if (strs.size() == 7)
       {
           //std::cout << "LINE[" << line << "]" << std::endl;

           double x,y,z;

           try
           {
               x = boost::lexical_cast<double>(strs[3]);
           }
           catch (boost::bad_lexical_cast e)
           {
               return;
           }

           try
           {
               y = boost::lexical_cast<double>(strs[5]);
           }
           catch (boost::bad_lexical_cast e)
           {
               return;
           }

           try
           {
               z = boost::lexical_cast<double>(strs[4]);
           }
           catch (boost::bad_lexical_cast e)
           {
               return;
           }

           xPos = -x;
           yPos = y;
           zPos = z;

           publish();

       }

    }

    // FRONT POS (used for angle/yaw)
    if (boost::starts_with(line, "OBJ1"))
    {
       std::vector<std::string> strs;
       boost::split(strs, line, boost::is_any_of(" \t"));
       if (strs.size() == 7)
       {
           //std::cout << "LINE[" << line << "]" << std::endl;

           double x,y,z;

           try
           {
               x = boost::lexical_cast<double>(strs[3]);
           }
           catch (boost::bad_lexical_cast e)
           {
               return;
           }

           try
           {
               y = boost::lexical_cast<double>(strs[5]);
           }
           catch (boost::bad_lexical_cast e)
           {
               return;
           }

           try
           {
               z = boost::lexical_cast<double>(strs[4]);
           }
           catch (boost::bad_lexical_cast e)
           {
               return;
           }

           // Same coordinate system as xPos
           x = -x;

           double idz = (z+0.05) - zPos;
           double idx = x - xPos;
           double idy = y - yPos;

           // ROLL (TODO)
           roll = 0.0;

           // PITCH
           pitch = -asin(idz/0.435);

           //std::cout << "LINE[" << line << "]" << std::endl;

           //std::cout << "PITCH = " << RAD2DEG(pitch) << std::endl;

           //pitch = 0.0;

           // YAW
           yaw = atan2(idy, idx);
           FIX_ANGLES(yaw);

           //std::cout << "ANGLE[" << yaw << "]" << std::endl;
           publish();

       }
   }

}

void VivePos::publish()
{
	ros::Time current_time = ros::Time::now();

  // ODOM
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom_combined";

  odom.pose.pose.position.x = xPos;
  odom.pose.pose.position.y = yPos;
  odom.pose.pose.position.z = zPos;

  odom.pose.pose.orientation.x = 0;
  odom.pose.pose.orientation.y = 0;
  odom.pose.pose.orientation.z = 0;
  odom.pose.pose.orientation.w = 1;
  odom_pub.publish(odom);

  // POSE
  //tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
  tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
  robot_pose.pose.orientation.x = q.x();
  robot_pose.pose.orientation.y = q.y();
  robot_pose.pose.orientation.z = q.z();
  robot_pose.pose.orientation.w = q.w();

  // Set this into the pose
  robot_pose.pose.position.x = xPos;
  robot_pose.pose.position.y = yPos;
  robot_pose.pose.position.z = zPos;

  robot_pose.header.frame_id = "base_link";
  robot_pose.header.stamp = ros::Time::now();
  pose_pub.publish(robot_pose);


  // Send the TF
  if (publishTf)
  {
    // Calculate the TF from the pose...
    transform.setOrigin(
                tf::Vector3(robot_pose.pose.position.x, robot_pose.pose.position.y, robot_pose.pose.position.z));
    tf::Quaternion q;
    tf::quaternionMsgToTF(robot_pose.pose.orientation, q);
    transform.setRotation(q);


      br.sendTransform(tf::StampedTransform(transform, current_time, "odom", "base_link"));
  }

}

} /* namespace Husqvarna */
