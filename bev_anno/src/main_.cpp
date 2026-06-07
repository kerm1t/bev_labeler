// 
// bev_anno
// Birdseye view annotation component for ADAS
//

#include <iostream>

#include "util/config_parser.h"

//#include "pointcloud.hpp" // has to be before draw.hpp -> 2do, fix!
//#include "draw.hpp"

// eCAL
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
// proto i/o
#include "pcl.pb.h"
#include "lane.pb.h"
#include "foxglove/LinePrimitive.pb.h"
#include "foxglove/PointCloud.pb.h"

// LLoFT
#include "pointcloud/pointcloud.hpp"
#include "pointcloud/pointcloud_synth.hpp"
#include "pointcloud/pointcloud_io.hpp"

#include <chrono>
float algo_runtime_ms; // 2do: encapsulate in frs.algo

// lloft objects initialized with an instance of pointcloud
lloft::pointcloud p;
lloft::pointcloud_synth psynth(p);
lloft::pointcloud_io pio(p);

// <algo_headers>

#include <mutex>
std::recursive_mutex m_SubscriberMutex; // ecal callback thread
bool bnewframe = false;

// 2do: above variables un-global and make available to below includes

#include "init.hpp"
#include "user.hpp"
#include "app_ecal.hpp"

//#include "demo_manager.hpp"

//demo_manager dman;

void publish_with_Mutex(eCAL::protobuf::CPublisher<pcl::PointCloud2>& publisher_frs) {
  const std::lock_guard<std::recursive_mutex> lock(m_SubscriberMutex);
  app_ecal::publish(publisher_frs);
}

void publish_fox_pointcloud_with_Mutex(eCAL::protobuf::CPublisher<foxglove::PointCloud>& publisher_gse_fox) {
  const std::lock_guard<std::recursive_mutex> lock(m_SubscriberMutex);
  app_ecal::publish_fox_pointcloud(publisher_gse_fox);
}

bool load_anno(std::string jsonFile) {
  try
  {
    std::ifstream ifs{ jsonFile };
    if (ifs.is_open())
    {
      nlohmann::json json(nlohmann::json::parse(ifs));
//      json.find("labels") != json.end() ? pio.labels = json["labels"].get<std::vector<lloft::label>>() : std::cerr << "No labels found in annotation file." << std::endl;
//      gseCfg = json.get<FrsCfgJson>();
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

struct Point2D { double x, y; };

struct Annotation {
  std::string cls;
  bool closed;
  std::vector<Point2D> points;
};

std::vector<Annotation> load_labels(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("Cannot open: " + path);

  nlohmann::json j;
  f >> j;

  std::vector<Annotation> result;
  for (const auto& ann : j.at("annotations")) {
    Annotation a;
    a.cls = ann.at("class").get<std::string>();
    a.closed = ann.value("closed", false);
    for (const auto& pt : ann.at("points"))
      a.points.push_back({ pt.at("x").get<double>(),
                          pt.at("y").get<double>() });
    result.push_back(std::move(a));
  }
  return result;
}


int main(int argc, char** argv)
{
  user::init_Cfg();

  app_ecal::init(argc, argv);

  int i_toggle = 1;
//  std::cout << "main.cpp: loading " << s[0] << std::endl;

  lloft::publish_ground = true;

//  load_anno("C:/GIT/BEV_labeler/labels.json");
  auto annotations = load_labels("C:/GIT/BEV_labeler/labels.json");
  for (const auto& ann : annotations) {
    std::cout << "[" << ann.cls << "]"
      << (ann.closed ? " (closed)" : " (open)")
      << "  " << ann.points.size() << " points\n";
    for (const auto& p : ann.points)
      std::cout << "  (" << p.x << ", " << p.y << ")\n";
  }

  // Loop
  bool close = false;
  do
  {
    if (bnewframe) {
// works -->      lloft::nvertices = 6000; // dummy
      auto t1 = std::chrono::high_resolution_clock::now();
// <algo_run>
//      frs::run_frs(p, p_freespace);
      auto t2 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> ms_double = t2 - t1;
      std::cout << "run_frs " << ms_double.count() << "ms\n";
      algo_runtime_ms = ms_double.count();

      lloft::nvertices = p.numpoints; // so here it works, but not in the eCal thread, hmmm
      
      publish_with_Mutex(app_ecal::publisher_frs);
      app_ecal::publish_Poly(app_ecal::publisher_poly);
      publish_fox_pointcloud_with_Mutex(app_ecal::publisher_fox);

      bnewframe = false;
    }

  } while (!close);

  return 0;
}
