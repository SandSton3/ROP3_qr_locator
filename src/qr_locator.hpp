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
// Message includes
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <dynamics_qr_msgs/QRCode.h>
#include <geometry_msgs/Point.h>
// C++ Standard includes
#include <string>
#include <math.h>
// Functional Defines
#define DEG_TO_RAD(X) ((X)*M_PI / 180.0) // Translate Degree to Radians
#define RAD_TO_DEG(X) ((X)*180.0 / M_PI) // Translate Radians to degrees
// Debug Defines
//#define DEBUG_ALIVE
//#define BEBUG_CALC_DIST
//#define DEBUG_PUBLISHER


struct Cam_params
{
    float _f{3.04};                 //focal lenght
    size_t _width{320};             //width in pixel
    float _fov_h{DEG_TO_RAD(70.0)}; //field of view horizontal in rad
    Cam_params() = default;
    ~Cam_params() = default;

    void calc_fov(float f, size_t width)
    {
        this->_f = f;
        this->_width = width;
        this->_fov_h = atan(_width / (2 * _f));
    }
    void print_params() const
    {
        ROS_INFO_STREAM("Params: f:" << _f << ", width:" << _width << ", fov_h:" << _fov_h);
    }
};

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
    ros::Subscriber _caminfo_sub; //subcribe to camer-info-info topic
    ros::Publisher _res_pub;      // result publisher
    ros::Timer _debug_timer;      // timer for debug messages
    /* Rest */
    std::string _pub_topic_name{"qr_location"}; //topic name, default: qr_location
    Cam_params _cam_params;                     // useful camera parameters
    void sync_cb(const sensor_msgs::LaserScanConstPtr &lidar_msg, const dynamics_qr_msgs::QRCodeConstPtr &qr_sub)
    {
        geometry_msgs::Point p = calc_point(lidar_msg, qr_sub);
        _res_pub.publish(p);
        #ifdef DEBUG_PUBLISHER
        ROS_INFO_STREAM("x: " << p.x << "| y:" << p.y << "| z:" << p.z);
        #endif
    }
#ifdef DEBUG_ALIVE
    void debug_cb(const ros::TimerEvent &e)
    {
        ROS_INFO_STREAM("Alive! @Time:" << e.current_real);
    }
#endif
public:
#pragma region CONSTRUCTORS
    Qr_locator(const std::string &lidar_topic_name, const std::string &qr_topic_name) : _nh(),                                        //create NodeHandle
                                                                                        _sync(MySyncPolicy(10), _lidar_sub, _qr_sub), //construct snc
                                                                                        _cam_params{}                                 /// init default camera parameter
    {
        _lidar_sub.subscribe(_nh, lidar_topic_name, 5);                          //Subcribe on lidar topic
        _qr_sub.subscribe(_nh, qr_topic_name, 5);                                //subcribe on qr topic
        _sync.registerCallback(boost::bind(&Qr_locator::sync_cb, this, _1, _2)); //synchronize messages
        _res_pub = _nh.advertise<geometry_msgs::Point>(_pub_topic_name, 5);
#ifdef DEBUG_ALIVE //Sebd Alive message
        _debug_timer = _nh.createTimer(ros::Duration(1.0), boost::bind(&Qr_locator::debug_cb, this, _1));
#endif
    };
    ~Qr_locator() = default;
#pragma endregion
#pragma region METHODS
    geometry_msgs::Point calc_point(const sensor_msgs::LaserScanConstPtr &lidar_msg, const dynamics_qr_msgs::QRCodeConstPtr &qr_msg) const
    {
/* Function to calculate the postion in reference to the camera for the x- and y-axis and in reffernce to the lidar in z-axis
*  The Z-Asxis can be be understand as the distance from the lidar to the qr-code. 
*  For a more accurat and more robust value a mean-value from a range of value is calulated (see parameter range in calc_dist_from_range_vec)
*/
        geometry_msgs::Point p;
        p.x = qr_msg->x;
        p.y = qr_msg->y;
        p.z = calc_dist_from_range_vec(lidar_msg, calc_angle_for_lidar(qr_msg));
        return p;
    }
    inline float const calc_angle_for_lidar(const dynamics_qr_msgs::QRCodeConstPtr &qr_msg) const
    {
        /* Function to calculate the angle in degree of the qr-code in reference to the lidar
    *
    * The Ratio between the width and the field of view horizontal equals the ratio of the x coordinate of the qr-code and the angel
    *  width/fov_h == xq/angle --> angle = fov_h*xq/widht
    *  The angle should be translated to the baseline --> angle - (fov_h/2)
    *  dnd translated in angle for the lidar --> 0°-360°
    */
        float angle{(_cam_params._fov_h / (float)_cam_params._width) * qr_msg->x}; // translate pixel to radians
        angle -= (_cam_params._fov_h / 2.0);                                       // Set Offset to Centernline
        angle += (angle < 0.0 ? 2 * M_PI : 0.0);                                   // Translate for Lidar (from 0 to 2Pi)
        return angle;
    }
    inline float const calc_dist_from_range_vec(const sensor_msgs::LaserScanConstPtr &lidar_msg, const float angle_rad, const int range = 2) const
    {
        /* Function to calculates a mean value of the distance from the range vector at a defined vector position
    * range is a parameter to define the range of positions to look for the meanvalue: range = 1 means that 1 postion left and 1 right
    */
        const size_t angle_pos = round(angle_rad / lidar_msg->angle_increment);
        if (range < 1)
        {
            return lidar_msg->ranges[angle_pos];
        }
        else
        {
            float dist_sum{lidar_msg->ranges[angle_pos]}; //add first elemt to sum
            size_t dist_cnt{1}; //set element counter to 1

            for (int shift{range * -1}; shift <= range; ++shift)
            {
                if (dist_cnt != 0) check_dist_and_add(dist_sum, lidar_msg->ranges[check_pos(angle_pos, shift, lidar_msg->ranges.size())], dist_cnt);
            }
        #ifdef DEBUG_CALC_DIST
           ROS_INFO_STREAM("pos: "<<angle_pos<<"first Value: "<<lidar_msg->ranges[angle_pos]<<"d_sum: " << dist_sum << "d_cnt: " << dist_cnt);
        #endif
            return dist_sum / (float)dist_cnt;
        }
    }
    inline size_t const check_pos(const size_t pos, const int shift, const size_t vec_length) const
    {
        /* Function to wrap the vector like a ringbuffer (also possible to use deque but not wort copy all entries)
            retuns the position with shift 
        */
        size_t ret_pos{0}; //return position
        if (pos == 0 && shift < 0)//Begin of the vector and negativ shift so pos is set to end minus shift
        { 
            ret_pos = vec_length - shift;
        }
        else if (pos == (vec_length - 1) && shift > 0) // End of the vector and shift to positiv
        { 
            ret_pos = (size_t)shift - 1;
        }
        else
        {
            ret_pos = pos+shift;
        }

        return ret_pos;
    }
    inline void check_dist_and_add(float &sum_dist, float cur_dist, size_t &count, const float max_variance = 0.01) const
    {
        /*Function to checks if a distance is plausible and add it to the sum if is
    */
        float diff {abs(cur_dist - (sum_dist / (float)count))}; //Differenc from current mean-value and new distance value
        if (diff < max_variance && cur_dist > 0.0) // Distancevariance is smaller as defined variance defaul = 0.001;
        { 
            ++count;
            sum_dist += cur_dist;
        }
    }
#pragma endregion
}; // class Qr_locator
