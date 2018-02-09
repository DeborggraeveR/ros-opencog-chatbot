
#ifndef ___ROS_OPENCOG_CHATBOT__CHATBOT_NODE_H___
#define ___ROS_OPENCOG_CHATBOT__CHATBOT_NODE_H___

#include "chatbot_api/chatbot_api.h"
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemeEval.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace ros_opencog
{
  namespace chatbot
  {
    class ChatbotNode
    {
    public:
      ChatbotNode(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle);
      ~ChatbotNode();
      
    private:
      void initializeOpenCog();
      void initializeSubscribers();
      void initializePublishers();
      void chatInputCallback(const std_msgs::String& message);
      
    private:
      ros::NodeHandle& _nodeHandle;
      ros::NodeHandle& _privateNodeHandle;
      ros::Subscriber _chatInputSubscriber;
      ros::Publisher _chatOutputPublisher;
      ros_opencog::chatbot::ChatbotApi* _chatbotApi;
      opencog::AtomSpace* _atomSpace;
      opencog::SchemeEval* _schemeEval;
      std::string _chatInputTopic;
      std::string _chatOutputTopic;
      
    };
  }
}

#endif // ___ROS_OPENCOG_CHATBOT__CHATBOT_NODE_H___
