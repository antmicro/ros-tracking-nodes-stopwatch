#ifndef CLOCK_HPP
#define CLOCK_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <vector>

class Clock
{
public:
    Clock() = delete;
    Clock(std::string description);
    void start();
    void stop();
    ros::Duration getDiff();
    std::string getDescription();
    std::vector<ros::Time> getRecords();
private:
    std::string m_description;
    ros::Time m_begin, m_end;
    std::vector<ros::Time> m_records;
};

#endif
