#include "clock.hpp"

#include <stdexcept>
#include <ros/ros.h>

Clock::Clock(std::string description)
{
    m_description = description;
    m_begin = m_end = ros::Time::now();
}

void Clock::start()
{
    if (m_begin > m_end)
    {
        m_begin = ros::Time::now();
        m_records.back() = m_begin;
        ROS_WARN("clock %s already started, reinitializing with %f", m_description.c_str(),
                m_begin.toSec());
    }
    else
    {
        m_begin = ros::Time::now();
        m_records.push_back(m_begin);
    }
}

void Clock::stop()
{
    if (m_begin < m_end)
    {
        m_end = ros::Time::now();
        ROS_WARN("clock %s already stopped", m_description.c_str());
    }
    else
    {
        m_end = ros::Time::now();
        m_records.push_back(m_end);
    }
}

ros::Duration Clock::getDiff()
{
    if (m_begin > m_end)
        throw std::runtime_error("Clock " + m_description + " in invalid state.");
    return m_end - m_begin;
}

std::string Clock::getDescription()
{
    return m_description;
}

std::vector<ros::Time> Clock::getRecords()
{
    return m_records;
}
