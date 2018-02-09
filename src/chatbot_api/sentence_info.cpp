
#include "chatbot_api/sentence_info.h"
#include <opencog/atomspace/AtomSpace.h>

ros_opencog::chatbot::SentenceInfo::SentenceInfo(const opencog::Handle& hSentence, const std::string& rawText)
  : _rawText(rawText)
  , _hSentenceNode(hSentence)
  , _hRawTextNode(opencog::Handle::UNDEFINED)
  , _hRawTextLink(opencog::Handle::UNDEFINED)
  , _hRawTextPredicateNode(opencog::Handle::UNDEFINED)
  , _hRawTextEvaluationLink(opencog::Handle::UNDEFINED)
{
  attachRawText(hSentence, rawText);
}

ros_opencog::chatbot::SentenceInfo::~SentenceInfo()
{
}

const std::string ros_opencog::chatbot::SentenceInfo::getRawText() const
{
  return _rawText;
}

opencog::Handle ros_opencog::chatbot::SentenceInfo::getSentenceNode() const
{
  return _hSentenceNode;
}

opencog::Handle ros_opencog::chatbot::SentenceInfo::getRawTextNode() const
{
  return _hRawTextNode;
}

opencog::Handle ros_opencog::chatbot::SentenceInfo::getRawTextLink() const
{
  return _hRawTextLink;
}

opencog::Handle ros_opencog::chatbot::SentenceInfo::getRawTextPredicateNode() const
{
  return _hRawTextPredicateNode;
}

opencog::Handle ros_opencog::chatbot::SentenceInfo::getRawTextEvaluationLink() const
{
  return _hRawTextEvaluationLink;
}

void ros_opencog::chatbot::SentenceInfo::attachRawText(const opencog::Handle& hSentence, const std::string& rawText)
{
  // Validate the sentence
  if (hSentence == opencog::Handle::UNDEFINED)
    return;
  
  // Get the atom space
  opencog::AtomSpace* atomSpace = hSentence->getAtomSpace();
  if (atomSpace == nullptr)
    return;
  
  // Create the raw text node and link it to the sentence
  _hRawTextNode = atomSpace->add_node(opencog::NODE, rawText);
  _hRawTextLink = atomSpace->add_link(opencog::LIST_LINK, hSentence, _hRawTextNode);
  
  // Create an evaluation link
  _hRawTextPredicateNode = atomSpace->add_node(opencog::PREDICATE_NODE, "sentence-rawtext");
  _hRawTextEvaluationLink = atomSpace->add_link(opencog::EVALUATION_LINK, _hRawTextPredicateNode, _hRawTextLink);
}
