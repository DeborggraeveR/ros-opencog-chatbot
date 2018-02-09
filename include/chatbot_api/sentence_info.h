
#ifndef ___ROS_OPENCOG_CHATBOT_API__SENTENCE_INFO_H___
#define ___ROS_OPENCOG_CHATBOT_API__SENTENCE_INFO_H___

#include <string>
#include <opencog/atoms/base/Handle.h>

namespace ros_opencog
{
  namespace chatbot
  {
    class SentenceInfo
    {
    public:
      SentenceInfo(const opencog::Handle& hSentence, const std::string& rawText);
      ~SentenceInfo();
      
    public:
      const std::string getRawText() const;
      opencog::Handle getSentenceNode() const;
      opencog::Handle getRawTextNode() const;
      opencog::Handle getRawTextLink() const;
      opencog::Handle getRawTextPredicateNode() const;
      opencog::Handle getRawTextEvaluationLink() const;
      
    private:
      void attachRawText(const opencog::Handle& hSentence, const std::string& rawText);
      
    private:
      std::string _rawText;
      opencog::Handle _hSentenceNode;
      opencog::Handle _hRawTextNode;
      opencog::Handle _hRawTextLink;
      opencog::Handle _hRawTextPredicateNode;
      opencog::Handle _hRawTextEvaluationLink;
      
    };
  }
}

#endif // ___ROS_OPENCOG_CHATBOT_API__SENTENCE_INFO_H___
