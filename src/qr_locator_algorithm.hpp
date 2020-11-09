#pragma onceS
#include "qr_locator_camparam.hpp"
#define RANGE 2 //range within the median of ranges is built (=2 means 2 left and 2 right from position)
#define DEVIATION 0.01 //allowed deviation so that a range is used to build mean range

namespace qr_locator
{
    namespace algorithm
    {
        inline double const convert_angle_for_lidar(const double angle, const sensor_msgs::LaserScanConstPtr &lidar_msg)
    {
        /* Function to convert angle in reference to the baseline in an angle for the laserscaner
        *  Maximum value = maximum of the lidar-angle, minimum value = minimum of the lidar-angle
        *  Return angle for Lidar in radians
        */
        return  ((angle < 0.0 ? lidar_msg->angle_max : lidar_msg->angle_min)+angle); // check if angle is negativ and translate to lidar-angel
    }
        double calc_angle_from_image(const dynamics_qr_msgs::QRCodeConstPtr &qr_msg,const qr_locator::Cam_params& cam_par)
    { /* Function to calculate the angel from the position of the QR-Code  in reference to the baseline of the camera 
       * returns the angle in radians
       * 
       * Uses Pinhole-Camera-model to calculate angel, to get the right sign atan2-function is is used
       */
        return {atan2(cam_par.get_cx() - qr_msg->x,cam_par.get_fx())};
    }
        inline size_t const check_pos(const size_t pos, const int shift, const size_t vec_length)
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
 
        inline void check_dist_and_add(double &sum_dist, double cur_dist, size_t &count, const double max_deviation = DEVIATION)
    {
        /*Function to checks if a distance is plausible and add it to the sum if is
    */
        double diff{abs(cur_dist - (sum_dist / (double)count))}; //Differenc from current mean-value and new distance value
        if (diff < max_deviation && cur_dist > 0.0)             // Distancedeviation is smaller as defined deviation default = 0.001;
        {
            ++count;
            sum_dist += cur_dist;
        }
    }
    
        inline double const calc_dist_from_range_vec(const sensor_msgs::LaserScanConstPtr &lidar_msg, const double angle_rad, const int range = RANGE)
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
                if (dist_cnt != 0) check_dist_and_add(dist_sum, lidar_msg->ranges[algorithm::check_pos(angle_pos, shift, lidar_msg->ranges.size())], dist_cnt);
            }
#ifdef DEBUG_CALC_DIST
            ROS_INFO_STREAM("pos: " << angle_pos << "first Value: " << lidar_msg->ranges[angle_pos] << "d_sum: " << dist_sum << "d_cnt: " << dist_cnt);
#endif
            return dist_sum / (double)dist_cnt;
        }
    }

    }; // namespace algorithm
}; // namespace qr_locator