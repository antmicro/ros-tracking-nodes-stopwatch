#ifndef CLOCK_HPP
#define CLOCK_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <vector>

/**
 * Class measuring one time interval - pair of two points in time
 * It uses ros::Time underneath
 */
class Clock
{
public:
    Clock() = delete; ///< Clocks need to have description
    Clock(std::string description); ///< Create clock with description
    void start(); ///< start time measurment (save current time as the first time point)
    void stop(); ///< stop time measurment (save current tiem as the second time point)
    ros::Duration getDiff(); ///< when clock is stopped, get last measurement
    std::string getDescription(); ///< return provided description
    std::vector<ros::Time> getRecords(); ///< return vector containing 
private:
    std::string m_description; ///< tells what the clock measures
    ros::Time m_begin, m_end; ///< timepoints representing the interval
    std::vector<ros::Time> m_records; ///< when the clock was toggled
};

#endif
