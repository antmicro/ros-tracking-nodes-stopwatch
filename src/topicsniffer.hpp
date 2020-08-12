#ifndef TOPICSNIFFER_HPP
#define TOPICSNIFFER_HPP

#include "stopwatch.hpp"

#include <stopwatch/registerPairService.h>
#include <topic_tools/shape_shifter.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <functional>
#include <mutex>

class TopicSniffer
{
public:
    TopicSniffer() = delete;
    TopicSniffer(const TopicSniffer&) = delete;
    TopicSniffer& operator=(const TopicSniffer&) = delete;
    TopicSniffer(Stopwatch& stopwatch);
private:
    class TopicHandler
    {
    public:
        TopicHandler(TopicSniffer& snif, const std::string& topic);
    private:
        class CallbackWrapper
        {
        public:
            CallbackWrapper() = delete;
            CallbackWrapper(TopicSniffer& snif, const std::string& topic);
            void callback(const topic_tools::ShapeShifter::ConstPtr& msg); 
        private:
            TopicSniffer& m_snif;
            std::string m_topic;
        };
        ros::Subscriber m_sub;
        CallbackWrapper m_wrap;
    };
    class Pair
    {
    public:
        Pair() = delete;
        Pair(const std::string& first_topic, const std::string& second_topic,
                unsigned first_topic_queue_size, const std::string& description,
                Stopwatch& stopwatch);
        void notify(const std::string& topic);
    private:
        std::string m_first_topic, m_second_topic;
        unsigned m_first_topic_queue_size;
        unsigned m_clock_id;
        Stopwatch& m_stopwatch;
        unsigned m_nQueuedMessages;
        bool m_working;
    };
    void advertiseServices();
    bool registerPair(stopwatch::registerPairService::Request& req,
            stopwatch::registerPairService::Response& res);
    void handleMessage(const std::string& topic);
    std::map<std::string, TopicHandler> m_topic_handlers;
    std::vector<Pair> m_pairs;
    std::map<std::string, std::vector<std::size_t>> m_topic_to_pair_id;
    Stopwatch& m_stopwatch;
    ros::ServiceServer m_register_pair_server;
};

#endif
