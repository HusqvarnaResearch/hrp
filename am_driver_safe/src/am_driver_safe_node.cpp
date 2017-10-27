/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#include <iostream>
#include <thread>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

/*
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>
*/

#include "am_driver_safe/automower_safe.h"
#include "am_driver_safe/automower_safe_states.h"

decision_making::RosEventQueue* eventQueue;
void stateMachineThread1();

int main( int argc, char** argv )
{
    ros::init(argc,argv, "am_driver_safe_node");
    ros_decision_making_init(argc, argv);

    ros::NodeHandle n;


    ros::Time lastTime;

    eventQueue = new decision_making::RosEventQueue();
    Husqvarna::AutomowerSafePtr am(new Husqvarna::AutomowerSafe(n,eventQueue));
    Husqvarna::ConnectDriverAndStates(am);

    double updateRate = am->GetUpdateRate();
    ros::Rate rate(updateRate);

    bool res = am->setup();
    if (!res)
    {
        return -1;
    }

    ros::Time last_time = ros::Time::now();

    std::thread fsmThread = std::thread(&stateMachineThread1);


    while( ros::ok() )
    {
        ros::Time current_time = ros::Time::now();
        ros::Duration dt = current_time - last_time;
        last_time = current_time;
        am->update(dt);

        ros::spinOnce();
        rate.sleep();

    }

    if ( fsmThread.joinable() )
    {
        fsmThread.join();
    }

    return 0;
}


void stateMachineThread1()
{
    ROS_INFO("AutomowerSafe StateMachine started. ");
    eventQueue->async_spin();

    Husqvarna::FsmAutoMowerSafeStates(NULL, eventQueue, "AutoMowerSafeStates");

    ROS_INFO("AutomowerSafe StateMachine stopped.");

}

