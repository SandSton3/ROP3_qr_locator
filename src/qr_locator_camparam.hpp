#pragma once
// Message includes
#include <sensor_msgs/CameraInfo.h>
// c++-standart includes
#include <numeric>

#define DEG_TO_RAD(X) ((X)*M_PI / 180.0) // Translate Degree to Radians
#define RAD_TO_DEG(X) ((X)*180.0 / M_PI) // Translate Radians to degrees

namespace qr_locator{
struct Cam_params final
{
    double _fx{265};                 // focal lenght in x [px]
    size_t _width{320};              // width in pixel
    double _fov_h{DEG_TO_RAD(60)};   // field of view horizontal in rad
    double _cx{165.0};               // baseline intersection x-axis
    bool _cam_params_set{false};     // set parameter: true-> is se / false-> is not set
    //Constructor
    Cam_params() = default;
    ~Cam_params() = default;

    void set_params(const sensor_msgs::CameraInfoConstPtr& cam_inf) 
    {
        /* Function to set camera-parameter
        */
        if (cam_inf->K.size() >=9 && std::accumulate(cam_inf->K.begin(),cam_inf->K.end(),0) > 0){ //check if sum of array > 0 so if there are vlauers
        _fx = cam_inf->K[0];
        _cx = cam_inf->K[2];
        _width = cam_inf->width > 0 ? cam_inf->width : _width;
        _fov_h = atan(_width / (_fx*2.0))*2.0; //calculate FOV from Pinhole-Model
        _cam_params_set = true;
        }
    }
    double get_fx() const{
        return {this->_fx};
    }
    double get_cx()const{
        return{this->_cx};
    }
};

};