#include "tictoc.hpp"

TicToc::TicToc(Stopwatch& stopwatch) : m_stopwatch(stopwatch)
{
    advertiseServices();
}

bool TicToc::tic(stopwatch::ticService::Request& req,
            stopwatch::ticService::Response& res)
{
    if (!m_stopwatch.findClock(req.id))
    {
        ROS_ERROR("No clock with id %lu", req.id);
        return 0;
    }
    return m_stopwatch.startClock(req.id);
}

bool TicToc::toc(stopwatch::tocService::Request& req,
            stopwatch::tocService::Response& res)
{
    if (!m_stopwatch.findClock(req.id))
    {
        ROS_ERROR("No clock with id %lu", req.id);
        return 0;
    }
    return m_stopwatch.stopClock(req.id);
}

bool TicToc::newClock(stopwatch::newClockService::Request& req,
            stopwatch::newClockService::Response& res)
{
    res.id = m_stopwatch.createClock(req.description);
    return 1;
}

void TicToc::advertiseServices()
{
    ros::NodeHandle nh;
    tic_server = nh.advertiseService("stopwatch/tic", &TicToc::tic, this);
    toc_server = nh.advertiseService("stopwatch/toc", &TicToc::toc, this);
    new_clock_server = nh.advertiseService("stopwatch/newClock", &TicToc::newClock, this);
}
