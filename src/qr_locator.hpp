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
#include <numeric>
#include <boost/range/numeric.hpp>
// Functional Defines
#define DEG_TO_RAD(X) ((X)*M_PI / 180.0) // Translate Degree to Radians
#define RAD_TO_DEG(X) ((X)*180.0 / M_PI) // Translate Radians to degrees
// Debug Defines
//#define DEBUG_ALIVE
//#define DEBUG_CALC_DIST
//#define DEBUG_CALC_LOCATION
//#define DEBUG_PUBLISHER
//#define DEBUG_CAM_PARAMS


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
    ros::Subscriber _caminfo_sub; // subcribe to camera-info-info topic
    ros::Publisher _res_pub;      // result publisher
    ros::Timer _debug_timer;      // timer for debug messages
    /* Rest */
    std::string _pub_topic_name{"/qr_codes/located"}; // topic name, default: /qr_codes/located
    Cam_params _cam_params;                     // useful camera parameters

#pragma region METHODS
    void sync_cb(const sensor_msgs::LaserScanConstPtr &lidar_msg, const dynamics_qr_msgs::QRCodeConstPtr &qr_msg)
    { /* Callback Function for synchronization
    */
        dynamics_qr_msgs::QRCode result = calc_location(lidar_msg, qr_msg);
#ifdef DEBUG_PUBLISHER
        ROS_INFO_STREAM("x: " << result.x << "| y:" << result.y << "| cd"<< (result.x + result.y));
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
        ROS_INFO_STREAM("Cam parameter set: |fx:" << _cam_params._fx << "| cx:"<<_cam_params._cx<< "| FOV:" <<_cam_params._fov_h);
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
        double angle_bl {calc_angle_from_image(qr_msg)};                // angle in reference to the baseline
        double angle_ls {convert_angle_for_lidar(angle_bl, lidar_msg)}; // angle in lidar-angles
        double dist {calc_dist_from_range_vec(lidar_msg, angle_ls)};    // distance in reference of the lidar
        result.x = cos(angle_bl)*dist; //x-Value of the qr-position (-1.0 cause of the inverse angle)
        result.y = sin(angle_bl)*dist; //y-Value of the qr-position (-1.0 cause of the inverse angle)
        #ifdef DEBUG_CALC_LOCATION
        ROS_INFO_STREAM("DEBUG: |abl:"<<RAD_TO_DEG(angle_bl)<<" |als: " << RAD_TO_DEG(angle_ls) << " |d: " << dist); 
        #endif
        return result;
    }
    inline double const convert_angle_for_lidar(const double angle, const sensor_msgs::LaserScanConstPtr &lidar_msg) const
    {
        /* Function to convert angle in reference to the baseline in an angle for the laserscaner
        *  Maximum value = maximum of the lidar-angle, minimum value = minimum of the lidar-angle
        *  Return angle for Lidar in radians
        */
        return  ((angle < 0.0 ? lidar_msg->angle_max : lidar_msg->angle_min)+angle); // check if angle is negativ and translate to lidar-angel
    }
    inline double const calc_angle_from_image(const dynamics_qr_msgs::QRCodeConstPtr &qr_msg) const
    { /* Function to calculate the angel from the position of the QR-Code  in reference to the baseline of the camera 
       * returns the angle in radians
       * 
       * Uses Pinhole-Camera-model to calculate angel, to get the right sign atan2-function is is used
       */
        return {atan2(_cam_params._cx-qr_msg->x,_cam_params._fx)};
    }
    inline double const calc_dist_from_range_vec(const sensor_msgs::LaserScanConstPtr &lidar_msg, const double angle_rad, const int range = 2) const
    {
        /* Function to calculates a mean value of the distance from the range vector at a defined vector position
        * range is a parameter to define the range of positions to look for the meanvalue: range = 1 means that 1 postion left and 1 right
        */
        const size_t angle_pos = round(angle_rad / lidar_msg->angle_increment);           //cant get smaller then zero (size_t)
        angle_pos >= lidar_msg->ranges.size() ? lidar_msg->ranges.size() - 1 : angle_pos; // if it hapens that pos is out of range
        if (range < 1)
        {
            return lidar_msg->ranges[angle_pos];
        }
        else
        {
            double dist_sum{lidar_msg->ranges[angle_pos]}; //add first elemt to sum
            size_t dist_cnt{1};                           //set element counter to 1

            for (int shift{range * -1}; shift <= range; ++shift)
            {
                if (dist_cnt != 0) check_dist_and_add(dist_sum, lidar_msg->ranges[check_pos(angle_pos, shift, lidar_msg->ranges.size())], dist_cnt);
            }
#ifdef DEBUG_CALC_DIST
            ROS_INFO_STREAM("pos: " << angle_pos << "first Value: " << lidar_msg->ranges[angle_pos] << "d_sum: " << dist_sum << "d_cnt: " << dist_cnt);
#endif
            return dist_sum / (double)dist_cnt;
        }
    }
    inline size_t const check_pos(const size_t pos, const int shift, const size_t vec_length) const
    {
        /* Function to wrap the vector like a ringbuffer (also possible to use deque but not wort copy all entries)
            retuns the position with shift 
        */
        size_t ret_pos{0};         //return position
        if (pos == 0 && shift < 0) //Begin of the vector and negativ shift so pos is set to end minus shift
        {
            ret_pos = vec_length - shift;
        }
        else if (pos == (vec_length - 1) && shift > 0) // End of the vector and shift to positiv
        {
            ret_pos = (size_t)shift - 1;
        }
        else
        {
            ret_pos = pos + shift;
        }

        return ret_pos;
    }
    inline void check_dist_and_add(double &sum_dist, double cur_dist, size_t &count, const double max_variance = 0.01) const
    {
        /*Function to checks if a distance is plausible and add it to the sum if is
    */
        double diff{abs(cur_dist - (sum_dist / (double)count))}; //Differenc from current mean-value and new distance value
        if (diff < max_variance && cur_dist > 0.0)             // Distancevariance is smaller as defined variance defaul = 0.001;
        {
            ++count;
            sum_dist += cur_dist;
        }
    }
#pragma endregion
public:
#pragma region METHODS
    Qr_locator(const std::string &lidar_topic_name, const std::string &qr_topic_name) : _nh(),                                        // create NodeHandle
                                                                                        _sync(MySyncPolicy(10), _lidar_sub, _qr_sub), // construct sync
                                                                                        _cam_params{}                                 // init default camera parameter
    {
        _lidar_sub.subscribe(_nh, lidar_topic_name, 5);                          // subcribe on lidar topic
        _qr_sub.subscribe(_nh, qr_topic_name, 5);                                // subcribe on qr topic
        _sync.registerCallback(boost::bind(&Qr_locator::sync_cb, this, _1, _2)); // synchronize messages
        _res_pub = _nh.advertise<dynamics_qr_msgs::QRCode>(_pub_topic_name, 5);  // publisher for result  
#ifdef DEBUG_ALIVE //Send alive message
        _debug_timer = _nh.createTimer(ros::Duration(1.0), boost::bind(&Qr_locator::debug_cb, this, _1));
#endif
    };
    ~Qr_locator() = default; // nothing extraordinary to do

    void set_cam_params_from_cam_info(const std::string& cam_info_topic){
        if (_cam_params._cam_params_set == false){
        _caminfo_sub = _nh.subscribe<sensor_msgs::CameraInfo>(cam_info_topic,1,&Qr_locator::cb_cam_info,this);
        }
    }
#pragma endregion
}; // class Qr_locator
