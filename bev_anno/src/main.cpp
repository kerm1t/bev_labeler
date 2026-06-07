// 
// bev_anno
// Birdseye view annotation component for ADAS
//

#include <iostream>

#include "include/util/config_parser.h"

//#include "pointcloud.hpp" // has to be before draw.hpp -> 2do, fix!
//#include "draw.hpp"


// LLoFT
#include "include/pointcloud/pointcloud.hpp"
#include "include/pointcloud/pointcloud_synth.hpp"
#include "include/pointcloud/pointcloud_io.hpp"

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
#include "app_algowrp.hpp" // before eCAL
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

int main(int argc, char** argv)
{
  user::init_Cfg();

  app_ecal::init(argc, argv);

  int i_toggle = 1;
  std::string filename = "C:/GIT/BEV_labeler/labels.json";
  std::cout << "main.cpp: loading " << filename << std::endl;
  algo::annotations = algo::load_labels(filename);
  algo::print_labels(algo::annotations);

  lloft::publish_ground = true;

  // Loop
  bool close = false;
  do
  {
// unschoen    if (bnewframe)
    {
      auto t1 = std::chrono::high_resolution_clock::now();
// <algo_run>
      algo::run_with_Mutex();
      auto t2 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> ms_double = t2 - t1;
      std::cout << "run_frs " << ms_double.count() << "ms\n";
      algo_runtime_ms = ms_double.count();

      lloft::nvertices = p.numpoints; // so here it works, but not in the eCal thread, hmmm
      
//      publish_with_Mutex(app_ecal::publisher_frs);
//      app_ecal::publish_Poly(app_ecal::publisher_poly);
      app_ecal::publish_fox_poly(app_ecal::publisher_poly);
//      publish_fox_pointcloud_with_Mutex(app_ecal::publisher_fox);

      bnewframe = false;
      _sleep(500); // ms
    }

  } while (!close);

  return 0;
}
