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
    /**
     * Only constructor with string argument is allowed
     */
    Clock() = delete; 

    /**
     * Create clock with description
     */
    Clock(std::string description); 

    /**
     * start time measurment (save current time as the first time point)
     */
    void start(); 

    /**
     * stop time measurment (save current tiem as the second time point)
     */
    void stop(); 

    /**
     * when clock is stopped, get last measurement
     */
    ros::Duration getDiff(); 

    /**
     * return provided description
     */
    std::string getDescription(); 

    /**
     * return vector containing records
     */
    std::vector<ros::Time> getRecords(); 

private:
    std::string m_description; ///< tells what the clock measures
    ros::Time m_begin{}, m_end{}; ///< timepoints representing the interval
    std::vector<ros::Time> m_records; ///< when the clock was toggled
};

#endif
