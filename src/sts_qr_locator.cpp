#pragma once
#include <ros/ros.h>
#include "qr_locator.hpp"



int main(int argc, char  *argv[])
{
    ros::init(argc,argv,"locator_node");
    Qr_locator my_loc("scan","qr_codes/detected");

    ros::spin();
    return 0;
}
