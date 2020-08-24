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

/**
 * This class provides registerPair service (you can learn about it in the docs)
 */
class TopicSniffer
{
public:
    TopicSniffer() = delete;
    TopicSniffer(const TopicSniffer&) = delete;
    TopicSniffer& operator=(const TopicSniffer&) = delete;
    /**
     * Create topic sniffer with given stopwatch.
     * Note that this patter almost always requires given stopwatch to live at least
     * as long as the topicsniffer, which references it
     * @param stopwatch stopwatch
     */
    TopicSniffer(Stopwatch& stopwatch);
private:
    /**
     * This class takes care of catching messages from one topic
     */
    class TopicHandler
    {
    public:
        /**
         * @param snif TopicSniffer used by the class
         * @param topic the topic to be handled
         */
        TopicHandler(TopicSniffer& snif, const std::string& topic);
    private: 
        /**
         * This class provides callback which is used to subscribe to a topic
         * and it notifies TopicSniffer that a message has been published
         */
        class CallbackWrapper
        {
        public:
            CallbackWrapper() = delete;
            /**
             * @param snif TopicSniffer notified about message being published
             * @param topic name of tracked topic
             */
            CallbackWrapper(TopicSniffer& snif, const std::string& topic);
            /**
             * The callback. It uses special ShapeShifter class. If you want
             * to learn about it check topic_tools package.
             * @param msg message
             */
            void callback(const topic_tools::ShapeShifter::ConstPtr& msg); 
        private:
            TopicSniffer& m_snif; ///< sniffer which will be notified
            std::string m_topic; ///< tracked topic name
        };
        ros::Subscriber m_sub; ///< ros subscriber
        CallbackWrapper m_wrap; ///< wrapper listening to the topic
    };
    /**
     * This class represents pair of topics that are listened to
     * and implements the logic
     */
    class Pair
    {
    public:
        Pair() = delete;
        /**
         * @param first_topic name of topic that starts measurment
         * @param second_topic name of topic that stops measurment
         * @param first_topic_queue_size queue size of the first topic
         * @param description description of the pair being measured
         */
        Pair(const std::string& first_topic, const std::string& second_topic,
                unsigned first_topic_queue_size, const std::string& description,
                Stopwatch& stopwatch);
        /**
         * Notifies that a message was published to a topic
         * @param topic topic name
         */
        void notify(const std::string& topic);
    private:
        std::string m_first_topic, m_second_topic; ///< respective topic names
        unsigned m_first_topic_queue_size{}; ///< queue size of the first topic
        unsigned m_clock_id{}; ///< id of used clock from Stopwatch
        Stopwatch& m_stopwatch; ///< stopwatch reference
        unsigned m_nQueuedMessages{}; ///< counts how many messages are queued up
        bool m_working{}; ///< is measured algorithm currently working
    };
    
    void advertiseServices(); ///< advertises ROS services
    bool registerPair(stopwatch::registerPairService::Request& req,
            stopwatch::registerPairService::Response& res); ///< ros service callback
    /**
     * When notified about a message published to a topic
     * Forward it to interested Pairs
     * @param topic topic name
     * @return if succeded
     */
    bool handleMessage(const std::string& topic);
    std::map<std::string, TopicHandler> m_topic_handlers; ///< container with topic handlers for each of tracked topics; key value is topic name
    std::vector<Pair> m_pairs; ///< container with registered pairs
    std::map<std::string, std::vector<std::size_t>> m_topic_to_pair_id; ///< for each topic store all pairs that listen to it
    Stopwatch& m_stopwatch; ///< stopwatch ref
    ros::ServiceServer m_register_pair_server; ///< ros service server
};

#endif
