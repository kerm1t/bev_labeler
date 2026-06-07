#ifndef USER_HPP
#define USER_HPP


namespace user {
  static int mx, my; // mouse-pos
  static int dmx, dmy; // mouse-delta

  // GUI output added
  std::vector<void (*)()> guifunctions;

  void init_Cfg() {
    std::cout << "\n" << "Loading json topic + cfg..." << std::endl;
    bool jsonStatus = false;
    // Read ecal topics from json --> NOT YET USED
    std::string json_path = "./conf/ecal/ecalTopics.json";
    std::string ecalJsonArgumentHeader = "--topics=";
    ecal_sim::ConfigParser::EcalTopicsJson jsonEcalTopics;
    jsonStatus = ecal_sim::ConfigParser::readEcalTopicsJson(jsonEcalTopics, json_path);
    if (jsonStatus == false)
    {
      std::cout << "\n" << "** Issue reading Ecal topics from Json. Using default topics **" << std::endl;
    }
    // Read gsm Cfg from json
    json_path = "./conf/frs.json";
    ecal_sim::ConfigParser::FrsCfgJson jsonFrsCfg;
    jsonStatus = ecal_sim::ConfigParser::readFrsCfgJson(jsonFrsCfg, json_path);
    if (jsonStatus == false)
    {
      std::cout << "\n" << "** Issue reading bev_anno cfg from Json. Using default Cfg **" << std::endl;
    }
    std::cout << "\n" << "FRS-Road-bounds [m] " << jsonFrsCfg.road_bounds << std::endl;

    lloft::publish_ground = jsonFrsCfg.publish_ground;
/*    frs::self_occlusion_az_from = jsonFrsCfg.self_occlusion_az_from;
    frs::self_occlusion_az_to = jsonFrsCfg.self_occlusion_az_to;
    frs::self_occlusion_distance = jsonFrsCfg.self_occlusion_az_to;
    // temp. -->
    frs::weirdseg_start = jsonFrsCfg.omit_seg_start;
    frs::weirdseg_stop = jsonFrsCfg.omit_seg_stop;
    //  frs::b_draw_frs = jsonFrsCfg.self_occlusion_az_to;
    //  frs::b_draw_unknown;
    //  frs::b_draw_polyline;
    //  self_occlusion_distance
    frs::fuzzfactor_to_exclude_points_at_road_boundary = jsonFrsCfg.road_bounds;

    // no json cfg (yet)
    frs::self_occlusion_distance = 5.0f;
    frs::b_draw_frs = true;
    frs::b_draw_unknown = false;
    frs::b_draw_polyline = true;
*/
  }
} // namespace user

#endif // USER_HPP
