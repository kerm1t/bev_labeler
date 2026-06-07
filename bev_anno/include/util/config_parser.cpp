#include "config_parser.h"
/*
bool ecal_sim::ConfigParser::readJson(JsonCfg& ecalTopics, const std::string jsonFile)
{
  bool retVal = false;
  try
  {
    std::ifstream ifs{ jsonFile };
    if (ifs.is_open())
    {
      nlohmann::json json(nlohmann::json::parse(ifs));
      ecalTopics = json.get<EcalTopicsJson>();
      retVal = true;
    }
    else
    {
      std::cerr << "Unable to open ecal topic configuration file : " << jsonFile << std::endl;
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error while reading ecal topic configuration file " << jsonFile << std::endl;
    std::cerr << "Exception thrown by JSON parser : " << e.what() << std::endl;
  }
  return retVal;
}
*/
bool ecal_sim::ConfigParser::readEcalTopicsJson(EcalTopicsJson& ecalTopics, const std::string jsonFile)
{
  bool retVal = false;
  try
  {
    std::ifstream ifs{ jsonFile };
    if (ifs.is_open())
    {
      nlohmann::json json(nlohmann::json::parse(ifs));
      ecalTopics = json.get<EcalTopicsJson>();
      retVal = true;
    }
    else
    {
      std::cerr << "Unable to open ecal topic configuration file : " << jsonFile << std::endl;
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error while reading ecal topic configuration file " << jsonFile << std::endl;
    std::cerr << "Exception thrown by JSON parser : " << e.what() << std::endl;
  }
  return retVal;
}

bool ecal_sim::ConfigParser::readFrsCfgJson(FrsCfgJson& gseCfg, std::string jsonFile)
{
  try
  {
    std::ifstream ifs{ jsonFile };
    if (ifs.is_open())
    {
      nlohmann::json json(nlohmann::json::parse(ifs));
      gseCfg = json.get<FrsCfgJson>();
      return true;
    }
    else
    {
      //std::cerr << "Unable to open gsm topic configuration file : " << jsonFile << std::endl;
      return false;
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error while reading gsm topic configuration file " << jsonFile << std::endl;
    std::cerr << "Exception thrown by JSON parser : " << e.what() << std::endl;
    return false;
  }
}
