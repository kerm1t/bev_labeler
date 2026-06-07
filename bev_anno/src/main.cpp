// 
// bev_anno
// Birdseye view annotation component for ADAS
//

#include <iostream>

#include "include/util/config_parser.h"

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

int main(int argc, char** argv)
{
  user::init_Cfg();

  app_ecal::init(argc, argv);

  // load labels from .json file
  int i_toggle = 1;
  std::string filename = "C:/GIT/BEV_labeler/labels.json";
  std::cout << "main.cpp: loading " << filename << std::endl;
  algo::annotations = algo::load_labels(filename);
  algo::print_labels(algo::annotations);

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
      
      app_ecal::publish_fox_poly(app_ecal::publisher_poly);

      bnewframe = false;
      _sleep(500); // ms
    }

  } while (!close);

  return 0;
}
