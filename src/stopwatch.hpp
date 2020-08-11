#ifndef STOPWATCH_HPP
#define STOPWATCH_HPP

#include "clock.hpp"

#include <stopwatch/saveRecordsService.h>

#include <ros/ros.h>
#include <string>
#include <vector>

class Stopwatch
{
public:
    Stopwatch();
    void advertiseServices();
    unsigned createClock(std::string name);
    void startClock(unsigned id);
    void stopClock(unsigned id);
    bool saveRecords(stopwatch::saveRecordsService::Request& req,
            stopwatch::saveRecordsService::Response& res);
private:
    std::vector<Clock> m_clocks;
    ros::ServiceServer save_records_service;
};

#endif
