/*
 * Copyright (c) 2017 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Jimmy Petersson
 *
 */

#include <iostream>
#include <ros/ros.h>

#include "am_vive_pos/vivepos.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "am_vive_pos");
    ros::NodeHandle n;

    Husqvarna::VivePosPtr parser(new Husqvarna::VivePos(n));

    parser->Run();
    return 0;
}
