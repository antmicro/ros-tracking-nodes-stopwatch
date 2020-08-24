#include "topicsniffer.hpp"

TopicSniffer::TopicSniffer(Stopwatch& stopwatch) : m_stopwatch(stopwatch)
{
    advertiseServices();
}

TopicSniffer::TopicHandler::TopicHandler(TopicSniffer& snif, const std::string& topic)
    : m_wrap(snif, topic)
{
    ros::NodeHandle nh;
    m_sub = nh.subscribe(topic.c_str(), 0u, &CallbackWrapper::callback, &m_wrap);
}

TopicSniffer::TopicHandler::CallbackWrapper::CallbackWrapper(TopicSniffer& snif,
        const std::string& topic)
    : m_snif(snif), m_topic(topic)
{}

void TopicSniffer::TopicHandler::CallbackWrapper::callback(
        const topic_tools::ShapeShifter::ConstPtr& msg)
{
    m_snif.handleMessage(m_topic);
}

TopicSniffer::Pair::Pair(const std::string& first_topic, const std::string& second_topic,
        unsigned first_topic_queue_size, const std::string& description, Stopwatch& stopwatch)
    : m_first_topic(first_topic), m_second_topic(second_topic), 
    m_first_topic_queue_size(first_topic_queue_size), m_stopwatch(stopwatch)
{
    m_clock_id = m_stopwatch.createClock(description);
    m_nQueuedMessages = 0;
    m_working = false;
}

void TopicSniffer::Pair::notify(const std::string& topic)
{
    if (!topic == m_first_topic && !topic == m_second_topic)
    {
        ROS_ERROR("[TopicSniffer] Critical error: %s does not belong to pair (%s, %s). "
                "Shutting down.", topic.c_str(), m_first_topic.c_str(),
                m_second_topic.c_str());
        ros::shutdown();
    }
    if (topic == m_first_topic)
        m_nQueuedMessages = (m_first_topic_queue_size ? std::min(m_nQueuedMessages + 1,
                            m_first_topic_queue_size) : m_nQueuedMessages + 1);
    else
    {
        if (!m_working) ROS_WARN("(%s, %s) logic error", m_first_topic.c_str(),
                m_second_topic.c_str());
        m_working = false;
        m_stopwatch.stopClock(m_clock_id);
    }
    if (!m_working && m_nQueuedMessages)
    {
        m_stopwatch.startClock(m_clock_id);
        m_working = 1, m_nQueuedMessages--;
    }
}

void TopicSniffer::advertiseServices()
{
    ros::NodeHandle nh;
    m_register_pair_server = nh.advertiseService("stopwatch/registerPair",
            &TopicSniffer::registerPair, this);
}

bool TopicSniffer::registerPair(stopwatch::registerPairService::Request& req,
            stopwatch::registerPairService::Response& res)
{
    auto id = m_pairs.size();
    m_pairs.emplace_back(req.first_topic, req.second_topic, req.queue_size, req.description,
            m_stopwatch);
    for (auto elem : {req.first_topic, req.second_topic})
    {
        if (m_topic_handlers.find(elem) == m_topic_handlers.end())
            m_topic_handlers.emplace(std::piecewise_construct,
                    std::forward_as_tuple(elem), std::forward_as_tuple(*this, elem));
        m_topic_to_pair_id[elem].push_back(id);
    }
    return true;
}

bool TopicSniffer::handleMessage(const std::string& topic)
{
    if (m_topic_to_pair_id.find(topic) == m_topic_to_pair_id.end())
    {
        ROS_ERROR("[TopicSniffer] Can't find topic %s", topic.c_str());
        return false;
    }
    for (auto& pair_id : m_topic_to_pair_id[topic])
        m_pairs[pair_id].notify(topic);
    return true;
}
