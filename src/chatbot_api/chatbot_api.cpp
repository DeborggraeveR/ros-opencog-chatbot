
#include "chatbot_api/chatbot_api.h"
#include <opencog/atoms/base/Atom.h>
#include <opencog/atoms/base/Link.h>

ros_opencog::chatbot::ChatbotApi::ChatbotApi(opencog::AtomSpace* atomSpace, opencog::SchemeEval* schemeEval)
  : _atomSpace(atomSpace)
  , _schemeEval(schemeEval)
{
  executeSchemeCommand("(use-modules (opencog))");
  executeSchemeCommand("(use-modules (opencog nlp))");
  executeSchemeCommand("(use-modules (opencog nlp relex2logic))");
}

ros_opencog::chatbot::ChatbotApi::~ChatbotApi()
{
}

const std::string ros_opencog::chatbot::ChatbotApi::chat(const std::string& message)
{
  ros_opencog::chatbot::SentenceInfoPtr sentence = nlpParse(message);
  if (sentence == nullptr)
    return "";
  
  relex2Logic(sentence);
  
  UtteranceType utteranceType = getSentenceUtteranceType(sentence);
  switch (utteranceType)
  {
    case UtteranceType::DeclarativeSpeechAct:
      return handleDeclarativeSpeechAct(sentence);
    case UtteranceType::InterrogativeSpeechAct:
      return handleInterrogativeSpeechAct(sentence);
    case UtteranceType::TruthQuerySpeechAct:
      return handleTruthQuerySpeechAct(sentence);
    case UtteranceType::ImperativeSpeechAct:
      return handleImperativeSpeechAct(sentence);
    default:
      return handleUnknownSpeechAct(sentence);
  }
}

const std::string ros_opencog::chatbot::ChatbotApi::handleDeclarativeSpeechAct(SentenceInfoPtr& sentence)
{
  return "You made a Declarative SpeechAct";
}

const std::string ros_opencog::chatbot::ChatbotApi::handleInterrogativeSpeechAct(SentenceInfoPtr& sentence)
{
  return "You made an Interrogative SpeechAct";
}

const std::string ros_opencog::chatbot::ChatbotApi::handleTruthQuerySpeechAct(SentenceInfoPtr& sentence)
{
  return "You asked a Truth Query";
}

const std::string ros_opencog::chatbot::ChatbotApi::handleImperativeSpeechAct(SentenceInfoPtr& sentence)
{
  return "You made a Imperative SpeechAct";
}

const std::string ros_opencog::chatbot::ChatbotApi::handleUnknownSpeechAct(SentenceInfoPtr& sentence)
{
  return "Sorry, I can't identify the speech act type";
}

ros_opencog::chatbot::SentenceInfoPtr ros_opencog::chatbot::ChatbotApi::nlpParse(const std::string& message)
{
  // Make sure the parsed sentence anchor is unlinked
  getNlpParsedSentenceHandle();
  
  // Parse the text using the relex server
  std::stringstream relexParseCommand;
  relexParseCommand << "(relex-parse \"" << message << "\")";
  executeSchemeCommand(relexParseCommand.str());
  
  // Get the sentence handle
  opencog::Handle hSentence = getNlpParsedSentenceHandle();
  if (hSentence == opencog::Handle::UNDEFINED)
  {
    opencog::logger().error("Failed to get the parsed sentence node");
    return nullptr;
  }
  
  // Create and return the result
  ros_opencog::chatbot::SentenceInfoPtr result = std::make_shared<ros_opencog::chatbot::SentenceInfo>(hSentence, message);
  return result;
}

void ros_opencog::chatbot::ChatbotApi::relex2Logic(SentenceInfoPtr& sentence)
{
  // Validate the sentence
  if (sentence == nullptr)
  {
    return;
  }
  
  // Get and validate the sentence handle
  opencog::Handle hSentence = sentence->getSentenceNode();
  if (hSentence == opencog::Handle::UNDEFINED)
  {
    return;
  }
  
  std::stringstream relex2LogicParseCommand;
  relex2LogicParseCommand << "(r2l-parse " << hSentence->to_short_string() << ")";
  executeSchemeCommand(relex2LogicParseCommand.str());
}

void ros_opencog::chatbot::ChatbotApi::executeSchemeCommand(const std::string& command)
{
  std::string result = _schemeEval->eval(command);
  result.erase(std::remove(result.begin(), result.end(), '\n'), result.end());
  if (result.length() > 0)
    opencog::logger().debug(result);
  _schemeEval->clear_pending();
}

void ros_opencog::chatbot::ChatbotApi::findLinkedAtoms(opencog::Handle handle, opencog::Type linkType, opencog::Type atomType, opencog::HandleSeq& handleSequence)
{
    // Clear the output
  handleSequence.clear();
  
  // Validate the handle
  if (handle == opencog::Handle::UNDEFINED)
  {
    return;
  }
  
  // Loop trough all links of the specified linkType
  opencog::IncomingSet links = handle->getIncomingSetByType(linkType);
  for (const opencog::LinkPtr& link : links)
  {
    // Loop tough all atoms in the link
    for (int i = 0; i < link->get_arity(); i++)
    {
      opencog::Handle hAtom = link->getOutgoingAtom(i);
      if (hAtom->get_type() == atomType)
      {
	handleSequence.push_back(hAtom);
      }
    }
  }
}

opencog::Handle ros_opencog::chatbot::ChatbotApi::getNlpParsedSentenceHandle()
{
  // Get the parsed sentence anchor node
  opencog::Handle hAnchor = _atomSpace->get_node(opencog::ANCHOR_NODE, "# New Parsed Sentence");
  if (hAnchor == opencog::Handle::UNDEFINED)
  {
    return opencog::Handle::UNDEFINED;
  }
  
  // Find the sentence nodes
  opencog::HandleSeq sentenceNodes;
  findLinkedAtoms(hAnchor, opencog::LIST_LINK/*opencog::classserver().getType("ListLink")*/, opencog::classserver().getType("SentenceNode"), sentenceNodes);
  
  // Get the anchor linked list
  opencog::IncomingSet links = hAnchor->getIncomingSetByType(opencog::LIST_LINK);
  if (links.size() > 0)
  {
    opencog::LinkPtr anchorLink = links[0];
    if (anchorLink != nullptr)
    {
      // Clear the anchor
      _atomSpace->extract_atom(anchorLink->get_handle());
    }
  }
  
  // Validate the number of sentences
  if (sentenceNodes.size() == 0)
  {
    return opencog::Handle::UNDEFINED;
  }
  
  // Return the first sentence
  return sentenceNodes[0];
}

ros_opencog::chatbot::UtteranceType ros_opencog::chatbot::ChatbotApi::getSentenceUtteranceType(SentenceInfoPtr& sentence)
{
  // Validate the sentence
  if (sentence == nullptr)
  {
    return UtteranceType::Invalid;
  }
  
  // Get and validate the sentence handle
  opencog::Handle hSentence = sentence->getSentenceNode();
  if (hSentence == opencog::Handle::UNDEFINED)
  {
    return UtteranceType::Invalid;
  }
  
  // Find the parse node
  opencog::HandleSeq parseNodes;
  findLinkedAtoms(hSentence, opencog::classserver().getType("ParseLink"), opencog::classserver().getType("ParseNode"), parseNodes);
  if (parseNodes.size() == 0)
  {
    return UtteranceType::Invalid;
  }
  
  // Find the intepretation node
  opencog::HandleSeq interpretationNodes;
  findLinkedAtoms(parseNodes[0], opencog::classserver().getType("InterpretationLink"), opencog::classserver().getType("InterpretationNode"), interpretationNodes);
  if (interpretationNodes.size() == 0)
  {
    return UtteranceType::Invalid;
  }
  
  // Find the define linguistic concept node
  opencog::HandleSeq definedLinguisticConceptNodes;
  findLinkedAtoms(interpretationNodes[0], opencog::classserver().getType("InheritanceLink"), opencog::classserver().getType("DefinedLinguisticConceptNode"), definedLinguisticConceptNodes);
  if (definedLinguisticConceptNodes.size() == 0)
  {
    return UtteranceType::Invalid;
  }
  
  // Translate the utterance name into the enumarable utterance type
  std::string utteranceName = definedLinguisticConceptNodes[0]->get_name();
  if (utteranceName == "DeclarativeSpeechAct")
  {
    return UtteranceType::DeclarativeSpeechAct;
  }
  else if (utteranceName == "TruthQuerySpeechAct")
  {
    return UtteranceType::TruthQuerySpeechAct;
  }
  else if (utteranceName == "InterrogativeSpeechAct")
  {
    return UtteranceType::InterrogativeSpeechAct;
  }
  else if (utteranceName == "ImperativeSpeechAct")
  {
    return UtteranceType::ImperativeSpeechAct;
  }
  else
  {
    return UtteranceType::Unknown;
  }
}
