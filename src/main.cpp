#include "stopwatch.hpp"
#include "tictoc.hpp"
#include "topicsniffer.hpp"

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stopwatch");
    ROS_INFO("Initialized");
    Stopwatch stopwatch;
    TicToc tictoc(stopwatch);
    TopicSniffer topicsniffer(stopwatch);
    ros::spin();
}
