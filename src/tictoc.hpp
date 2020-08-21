#ifndef TICTOC_HPP
#define TICTOP_HPP

#include "stopwatch.hpp"

#include <stopwatch/ticService.h>
#include <stopwatch/tocService.h>
#include <stopwatch/newClockService.h>
#include <ros/ros.h>

/**
 * This class provides tic toc and newClock services (you can learn about them in docs)
 */
class TicToc
{
public:
    TicToc() = delete; ///< Requires stopwatch
    TicToc(Stopwatch& stopwatch);
    bool tic(stopwatch::ticService::Request& req,
                stopwatch::ticService::Response& res); ///< tic service callback
    bool toc(stopwatch::tocService::Request& req,
                stopwatch::tocService::Response& res); ///< toc service callback
    bool newClock(stopwatch::newClockService::Request& req,
                stopwatch::newClockService::Response& res); ///< newClock service callback
private:
    void advertiseServices(); ///< advertise services
    ros::ServiceServer tic_server, toc_server, new_clock_server; ///< ros service servers
    Stopwatch& m_stopwatch; ///< stopwatch reference
};


#endif
