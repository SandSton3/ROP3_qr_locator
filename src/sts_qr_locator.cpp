#pragma once
#include <ros/ros.h>
#include "qr_locator.hpp"



int main(int argc, char  *argv[])
{
    ros::init(argc,argv,"locator_node"); // init ros
    Qr_locator my_loc("scan","qr_codes/detected"); // create Qr-locator-objekt
    my_loc.set_cam_params_from_cam_info("/camera/camera_info"); // set camera parameter
    ros::spin(); // run till ros stops

    return 0; // No Runtime error
}
