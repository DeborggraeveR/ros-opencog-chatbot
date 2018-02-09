
#include "ros_opencog_chatbot/chatbot_node.h"

ros_opencog::chatbot::ChatbotNode::ChatbotNode(ros::NodeHandle& nodeHandle, ros::NodeHandle& privateNodeHandle)
  : _nodeHandle(nodeHandle)
  , _privateNodeHandle(privateNodeHandle)
  , _atomSpace(nullptr)
  , _schemeEval(nullptr)
  , _chatbotApi(nullptr)
{
  _privateNodeHandle.param<std::string>("input_topic", _chatInputTopic, "opencog/chat_input");
  _privateNodeHandle.param<std::string>("output_topic", _chatOutputTopic, "opencog/chat_output");
  
  std::vector<std::string> scm_scripts;
  _privateNodeHandle.getParam("scm_scripts", scm_scripts);
  ROS_INFO_STREAM("SCRIPT COUNT: " << scm_scripts.size());
  for (int i = 0; i < scm_scripts.size(); i++)
    ROS_INFO_STREAM("RUN SCRIPT: " << scm_scripts[i]);
  
  initializeOpenCog();
  initializeSubscribers();
  initializePublishers();
}

ros_opencog::chatbot::ChatbotNode::~ChatbotNode()
{
  if (_chatbotApi != nullptr)
    delete _chatbotApi;
  if (_schemeEval != nullptr)
    delete _schemeEval;
  if (_atomSpace != nullptr)
    delete _atomSpace;
}

void ros_opencog::chatbot::ChatbotNode::initializeOpenCog()
{
  _atomSpace = new opencog::AtomSpace();
  _schemeEval = new opencog::SchemeEval(_atomSpace);
  _chatbotApi = new ros_opencog::chatbot::ChatbotApi(_atomSpace, _schemeEval);
}

void ros_opencog::chatbot::ChatbotNode::initializeSubscribers()
{
  _chatInputSubscriber = _nodeHandle.subscribe(_chatInputTopic, 1, &ChatbotNode::chatInputCallback, this);
  
  ROS_INFO_STREAM("Subscribed input topic: " << _chatInputTopic);
}

void ros_opencog::chatbot::ChatbotNode::initializePublishers()
{
  _chatOutputPublisher = _nodeHandle.advertise<std_msgs::String>(_chatOutputTopic, 1, true);
  
  ROS_INFO_STREAM("Publishing output topic: " << _chatOutputTopic);
}

void ros_opencog::chatbot::ChatbotNode::chatInputCallback(const std_msgs::String& message)
{
  std_msgs::String response;
  response.data = _chatbotApi->chat(message.data);
  _chatOutputPublisher.publish(response);
}

int main(int argc, char** argv)
{
  // Initialize the node
  ros::init(argc, argv, "opencog_chatbot");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  
  // Setup the chatbot node
  ros_opencog::chatbot::ChatbotNode chatbotNode(nh, private_nh);
  
  // Run the mainloop
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}