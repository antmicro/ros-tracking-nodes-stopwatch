#ifndef STOPWATCH_HPP
#define STOPWATCH_HPP

#include "clock.hpp"

#include <stopwatch/saveRecordsService.h>

#include <ros/ros.h>
#include <string>
#include <vector>

/**
 * Class managing clocks, providing interface to access them and saving their records
 */
class Stopwatch
{
public:
    Stopwatch(); ///< default constructor
    /**
     * Creates a clock and returns its id, used to reference it in future
     * @param name clock name
     * @return id
     */
    unsigned createClock(std::string name); 
    /**
     * Starts clock with given id
     * @param id clock's id
     * @return true on success false otherwise
     */
    bool startClock(unsigned id);
    /**
     * Stops clock with given id
     * @param id clock's id
     * @return true on success false otherwise
     */
    bool stopClock(unsigned id);
    /**
     * Tell if clock with given id exists
     * @param id clock's id
     * @return true if clock was found, false otherwise
     */
    bool findClock(unsigned id);
    /**
     * Service callback for saving gathered records to a file
     * @param req ros service request
     * @param res ros service response
     * @return true on success false otherwise
     */
    bool saveRecords(stopwatch::saveRecordsService::Request& req,
            stopwatch::saveRecordsService::Response& res);
private:
    void advertiseServices(); ///< advertise ros services
    std::vector<Clock> m_clocks; ///< handled clocks
    ros::ServiceServer save_records_service; ///< ros service for saving records
};

#endif
