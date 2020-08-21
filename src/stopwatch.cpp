#include "stopwatch.hpp"

#include <ros/ros.h>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <cassert>
#include <string>

Stopwatch::Stopwatch()
{
    advertiseServices();
}

void Stopwatch::advertiseServices()
{
    ros::NodeHandle nh;
    save_records_service = nh.advertiseService("stopwatch/saveRecords",
            &Stopwatch::saveRecords, this);
}

unsigned Stopwatch::createClock(std::string name)
{
    m_clocks.emplace_back(name);
    ROS_INFO("Clock created; id %lu; name %s", m_clocks.size() - 1, name.c_str());
    return m_clocks.size() - 1;
}

bool Stopwatch::startClock(unsigned id)
{
    if (id >= m_clocks.size())
    {
        ROS_ERROR("clock id %d out of range\n", id);
        return false;
    }
    m_clocks[id].start();
    return true;
}

bool Stopwatch::stopClock(unsigned id)
{
    if (id >= m_clocks.size())
    {
        ROS_ERROR("clock id %d out of range\n", id);
        return false;
    }
    m_clocks[id].stop();
    return true;
}

bool Stopwatch::findClock(unsigned id)
{
    return (id < m_clocks.size());
}

bool Stopwatch::saveRecords(stopwatch::saveRecordsService::Request& req,
                            stopwatch::saveRecordsService::Response& res)
{
    std::string filename = req.out_path;
    std::ofstream out(filename);
    if (!out)
    {
        ROS_ERROR("%s", ("Could not open file " + filename + " for writing.").c_str());
        return false;
    }
    ROS_INFO("Saving records to file");
    out << std::setprecision(15) << std::fixed;
    for (auto clock : m_clocks)
    {
        out << clock.getDescription();
        for (auto time : clock.getRecords())
            out << ',' << time.toSec();
        out << '\n';
    }
    ROS_INFO("Records saved");
    return true;
}
