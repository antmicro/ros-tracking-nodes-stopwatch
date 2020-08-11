#include "stopwatch.hpp"

#include <ros/ros.h>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <cassert>
#include <string>

// we'll need some class to keep data about the pairs
// we need to keep clocks somewhere probably some map id->clock (can't use [])
// we need to keep subs and pass info to interested pair keepers so probably some
// topic->set of pair keepers

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
    return m_clocks.size() - 1;
}

void Stopwatch::startClock(unsigned id)
{
    if (id >= m_clocks.size())
    {
        ROS_ERROR("clock id %d out of range\n", id);
        return;
    }
    m_clocks[id].start();
}

void Stopwatch::stopClock(unsigned id)
{
    if (id >= m_clocks.size())
    {
        ROS_ERROR("clock id %d out of range\n", id);
        return;
    }
    m_clocks[id].stop();
}

bool Stopwatch::saveRecords(stopwatch::saveRecordsService::Request& req,
                            stopwatch::saveRecordsService::Response& res)
{
    std::string filename = req.out_path;
    std::ofstream out(filename);
    if (!out)
    {
        ROS_ERROR("%s", ("Could not open file " + filename + " for writing.").c_str());
        return 0;
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
    return 1;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stopwatch");
    ROS_INFO("Initialized");
}
