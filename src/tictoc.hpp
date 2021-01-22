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
    /**
     * Only constructor with an argument is allowed
     */
    TicToc() = delete; 

    /**
     * The only constructor, first argument is stopwatch. Note that the stopwatch has to outlive the constructed TicToc.
     */
    TicToc(Stopwatch& stopwatch);

    /**
     * tic service callback
     */
    bool tic(stopwatch::ticService::Request& req,
            stopwatch::ticService::Response& res); 

    /**
     * toc service callback
     */
    bool toc(stopwatch::tocService::Request& req,
            stopwatch::tocService::Response& res); 

    /**
     * newClock service callback
     */
    bool newClock(stopwatch::newClockService::Request& req,
                stopwatch::newClockService::Response& res); 

private:
    void advertiseServices(); ///< advertise services
    ros::ServiceServer tic_server, toc_server, new_clock_server; ///< ros service servers
    Stopwatch& m_stopwatch; ///< stopwatch reference
};


#endif
