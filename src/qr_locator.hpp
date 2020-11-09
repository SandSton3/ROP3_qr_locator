/* ############  QR-Locator Class ###########
** Is used to fusion a Lidar- with a Camera-signal to calculate the position of the QR-Code in reference to the lidar
** Author:       Sandro Steinhuber
** Personalnr.:  S1910828004
** Course:       ROP3
** Year:         2020
** Feel free to use
*/
#pragma once
// ROS includes
#include <ros/ros.h>
// Message Filters includes
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
// ROS-Message includes
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamics_qr_msgs/QRCode.h>
// C++-Standard includes
#include <string>
#include <math.h>
// Package internal includes
#include "qr_locator_algorithm.hpp"
#include "qr_locator_camparam.hpp"
// Debug Defines
//#define DEBUG_ALIVE
//#define DEBUG_CALC_DIST
//#define DEBUG_CALC_LOCATION
#define DEBUG_PUBLISHER
//#define DEBUG_CAM_PARAMS
//#define DEBUG_PARAMS

namespace qr_locator{

class Qr_locator final
{
private:
    /*  Message Filter member */
    message_filters::Subscriber<sensor_msgs::LaserScan> _lidar_sub; // subcriber to lidar-topic
    message_filters::Subscriber<dynamics_qr_msgs::QRCode> _qr_sub;  // subcriber to qr-topic
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, dynamics_qr_msgs::QRCode> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> _sync; //synchtonizer
    /* ROS-Specific */
    ros::NodeHandle _nh;          // handle to roscore
    ros::Subscriber _caminfo_sub; // subcribe to camera-info-info topic
    ros::Publisher _res_pub;      // result publisher
    ros::Timer _debug_timer;      // timer for debug messages
    /* camera parameter */
    qr_locator::Cam_params _cam_params;                     // useful camera parameters
    /* Parmaters */
    std::string _cam_info_topic;    // name of the camera-info topic
    std::string _qr_detector_topic; // name of the qr-detector topic
    std::string _scan_topic;        // name of the scan topic 
    std::string _result_topic;      // name of the result topic
#pragma region CALLBACKS
    void sync_cb(const sensor_msgs::LaserScanConstPtr &lidar_msg, const dynamics_qr_msgs::QRCodeConstPtr &qr_msg)
    { /* Callback Function for synchronization
    */
        dynamics_qr_msgs::QRCode result = calc_location(lidar_msg, qr_msg);
#ifdef DEBUG_PUBLISHER
        ROS_INFO_STREAM("Results:" <<std::right<< std::fixed<<std::showpos<<
        "| x: "   << result.x << 
        "| y:"  << result.y <<
        "| cd: " << (result.x + result.y));
#endif
        if (abs(result.x +result.y) > 0.0){ //chessboard distance has to be valid
            _res_pub.publish(result);
        } 
        else{
            ROS_INFO_STREAM("Distance from Lidar < 0.0 ");
        }
    }

#ifdef DEBUG_ALIVE
    void debug_cb(const ros::TimerEvent &e)
    {
        ROS_INFO_STREAM("Alive! @Time:" << e.current_real);
    #ifdef DEBUG_CAM_PARAMS
        ROS_INFO_STREAM("CAM params: |fx:" << _cam_params._fx << "| cx:"<<_cam_params._cx<< "| FOV:" <<_cam_params._fov_h );
    #endif
    }
#endif
    void cb_cam_info(const sensor_msgs::CameraInfoConstPtr& cam_msg){
        /* Callback function to set camera-parameter
        */
        _cam_params.set_params(cam_msg); // set camera parameter
        ROS_INFO_STREAM("Cam parameter set: |fx:" <<std::left<<_cam_params._fx << "| cx:"<<_cam_params._cx<< "| FOV:" <<_cam_params._fov_h);
        _caminfo_sub.shutdown(); // unsubcribe callback
    }

    dynamics_qr_msgs::QRCode calc_location(const sensor_msgs::LaserScanConstPtr &lidar_msg, const dynamics_qr_msgs::QRCodeConstPtr &qr_msg) const
    {
        /* Function to calculate the postion in reference to the camera for the x- and y-axis and in reffernce to the lidar in z-axis
        *  The Z-Asxis can be be understand as the distance from the lidar to the qr-code. 
        *  For a more accurat and more robust value a mean-value from a range of value is calulated (see parameter range in calc_dist_from_range_vec)
        */
        dynamics_qr_msgs::QRCode result;
        result.header.frame_id = "base_scan"; // coordinates in refernce to the lidar-frame
        result.data = qr_msg->data; // take data from the qr-message
        const double angle_bl {algorithm::calc_angle_from_image(qr_msg,_cam_params)};     // angle in reference to the baseline
        double angle_ls {algorithm::convert_angle_for_lidar(angle_bl, lidar_msg)}; // angle in lidar-angles
        double dist {algorithm::calc_dist_from_range_vec(lidar_msg, angle_ls)};    // distance in reference of the lidar
        result.x = cos(angle_bl)*dist; //x-Value of the qr-position (-1.0 cause of the inverse angle)
        result.y = sin(angle_bl)*dist; //y-Value of the qr-position (-1.0 cause of the inverse angle)
        #ifdef DEBUG_CALC_LOCATION
        ROS_INFO_STREAM("DEBUG: |abl:"<<RAD_TO_DEG(angle_bl)<<" |als: " << RAD_TO_DEG(angle_ls) << " |d: " << dist); 
        #endif
        return result;
    }

    void init_params(){
        _nh.getParam("qr_locator/p_cam_info_topic"   ,_cam_info_topic);   
        _nh.getParam("qr_locator/p_qr_detector_topic",_qr_detector_topic);
        _nh.getParam("qr_locator/p_scan_topic"       ,_scan_topic);       
        _nh.getParam("qr_locator/p_result_topic"     ,_result_topic);
        #ifdef DEBUG_PARAMS
        ROS_INFO_STREAM(
            ">>>  Parameter set  <<<" << std::endl <<
            "Cam-Info-Topic: "      << _cam_info_topic    <<std::endl <<
            "Qr-Detector-Topic: "   << _qr_detector_topic <<std::endl <<
            "Scan-Topic: "          << _scan_topic        <<std::endl <<
            "Result-Topic: "        << _result_topic;
        );     
        #endif
    }
#pragma endregion
public:
#pragma region METHODS

    Qr_locator():_nh(),
                _sync(MySyncPolicy(10),_lidar_sub,_qr_sub),
                _cam_params{}
    {
        init_params();
        _lidar_sub.subscribe(_nh,_scan_topic, 5);                                   // subcribe on lidar top
        _qr_sub.subscribe(_nh,_qr_detector_topic, 5);                               // subcribe on qr topic
        _sync.registerCallback(boost::bind(&Qr_locator::sync_cb, this, _1, _2));    // synchronize messages
        _res_pub = _nh.advertise<dynamics_qr_msgs::QRCode>(_result_topic, 5);       // publisher for result
#ifdef DEBUG_ALIVE //Send alive message
        _debug_timer = _nh.createTimer(ros::Duration(1.0), boost::bind(&Qr_locator::debug_cb, this, _1));
#endif
    };
    ~Qr_locator() = default; // nothing extraordinary to do

    void set_cam_params_from_cam_info(){
        /* Function to (try) to set internal camera-parameter from the  specified topic (see paramter camer_info_topic)
        */
        if (_cam_params._cam_params_set == false){
        _caminfo_sub = _nh.subscribe<sensor_msgs::CameraInfo>(_cam_info_topic,1,&Qr_locator::cb_cam_info,this);
        }
    }
#pragma endregion
}; // class Qr_locator

};// namespace qr:locator