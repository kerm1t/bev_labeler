#ifndef config_parser_h__
#define config_parser_h__

#include <fstream>
#include <iostream>
#include "nlohmann/json.hpp"

#define HRL_PCL_TOPIC "meta_pcl"
#define HRL_RSC_TOPIC "rsc"
#define HRL_GEN_TOPIC "gen"
#define HRL_GSM_TOPIC "gsm"
//#define HRL_GSM_DEBUG_TOPIC "gsm_debug"
#define HRL_FRS_TOPIC "frs_out"
#define HRL_SLAM_TOPIC "slam"
#define HRL_EMO_TOPIC "emo"
#define HRL_LOCA_TOPIC "loca"
#define HRL_TP_TOPIC "tp"
#define HRL_ADMA_TOPIC "adma"

namespace ecal_sim
{
	class ConfigParser
	{
	public:
		// https://stackoverflow.com/questions/979211/struct-inheritance-in-c
		struct JsonCfg
		{

		};

		struct EcalTopicsJson : JsonCfg
		{
			std::string meta_pca_topic{ HRL_PCL_TOPIC };
			std::string rsc_topic{ HRL_RSC_TOPIC };
			std::string gsm_topic{ HRL_GSM_TOPIC };

			NLOHMANN_DEFINE_TYPE_INTRUSIVE(EcalTopicsJson, meta_pca_topic, rsc_topic, gsm_topic);
		};

		struct FrsCfgJson : JsonCfg
		{
			int publish_ground{ true };       // atm GEN needs Poiunt cloud to be fully transferred
			int self_occlusion_az_from{ 0 };  // Ego vehicle occlusion of sensor FoV
			int self_occlusion_az_to{ 29 };   // Ego vehicle occlusion of sensor FoV
			int omit_seg_start{ 0 };          // Omit these segments starting with ...
			int omit_seg_stop{ 2 };           // Omit these segments stopping with ...
			float self_occlusion_distance{ 5.0 };  // Ego vehicle occlusion of sensor FoV
			float road_bounds{ 2.75 };        // Noise points intensity threshold. The reflected points have relatively small intensity than others.
			NLOHMANN_DEFINE_TYPE_INTRUSIVE(FrsCfgJson, publish_ground, self_occlusion_az_from, self_occlusion_az_to, omit_seg_start, omit_seg_stop, self_occlusion_distance, road_bounds);
		};

		/// Read the eCal topics json file
		/// @param[out] ecalTopics  The struct holding the ecal topic names.
		/// @param[in]  jsonFile    Path to json file.
		static bool readEcalTopicsJson(EcalTopicsJson& ecalTopics, const std::string jsonFile);

		static bool readFrsCfgJson(FrsCfgJson& gseCfg, const std::string jsonFile);
	};
}
#endif // config_parser_h__
