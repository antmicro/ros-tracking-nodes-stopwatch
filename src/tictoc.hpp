#ifndef TICTOC_HPP
#define TICTOP_HPP

#include "stopwatch.hpp"

#include <stopwatch/ticService.h>
#include <stopwatch/tocService.h>
#include <stopwatch/newClockService.h>
#include <ros/ros.h>

class TicToc
{
public:
    TicToc() = delete;
    TicToc(Stopwatch& stopwatch);
    bool tic(stopwatch::ticService::Request& req,
                stopwatch::ticService::Response& res);
    bool toc(stopwatch::tocService::Request& req,
                stopwatch::tocService::Response& res);
    bool newClock(stopwatch::newClockService::Request& req,
                stopwatch::newClockService::Response& res);
private:
    void advertiseServices();
    ros::ServiceServer tic_server, toc_server, new_clock_server;
    Stopwatch& m_stopwatch;
};


#endif
