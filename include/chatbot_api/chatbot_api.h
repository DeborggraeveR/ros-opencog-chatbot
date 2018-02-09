
#ifndef ___ROS_OPENCOG_CHATBOT_API__CHATBOT_API_H___
#define ___ROS_OPENCOG_CHATBOT_API__CHATBOT_API_H___

#include "chatbot_api/sentence_info.h"
#include <string>
#include <memory>
#include <opencog/atoms/base/types.h>
#include <opencog/atoms/base/Handle.h>
#include <opencog/atoms/base/ClassServer.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/guile/SchemeEval.h>

namespace ros_opencog
{
  namespace chatbot
  {
    typedef std::shared_ptr<SentenceInfo> SentenceInfoPtr;
    
    enum class UtteranceType : uint8_t
    {
      DeclarativeSpeechAct = 0,
      InterrogativeSpeechAct = 1,
      TruthQuerySpeechAct = 2,
      ImperativeSpeechAct = 3,
      Unknown = 0xFE,
      Invalid = 0xFF,
    };
    
    class ChatbotApi
    {
    public:
      ChatbotApi(opencog::AtomSpace* atomSpace, opencog::SchemeEval* schemeEval);
      ~ChatbotApi();
      
    public:
      const std::string chat(const std::string& message);
      const std::string handleDeclarativeSpeechAct(SentenceInfoPtr& sentence);
      const std::string handleInterrogativeSpeechAct(SentenceInfoPtr& sentence);
      const std::string handleTruthQuerySpeechAct(SentenceInfoPtr& sentence);
      const std::string handleImperativeSpeechAct(SentenceInfoPtr& sentence);
      const std::string handleUnknownSpeechAct(SentenceInfoPtr& sentence);
      ros_opencog::chatbot::SentenceInfoPtr nlpParse(const std::string& message);
      void relex2Logic(SentenceInfoPtr& sentence);
      void executeSchemeCommand(const std::string& command);
      void findLinkedAtoms(opencog::Handle handle, opencog::Type linkType, opencog::Type atomType, opencog::HandleSeq& handleSequence);
      opencog::Handle getNlpParsedSentenceHandle();
      UtteranceType getSentenceUtteranceType(SentenceInfoPtr& sentence);
      
    private:
      opencog::AtomSpace* _atomSpace;
      opencog::SchemeEval* _schemeEval;
      
    };
  }
}

#endif // ___ROS_OPENCOG_CHATBOT_API__CHATBOT_API_H___
