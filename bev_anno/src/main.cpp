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
float algo_runtime_ms; // 2do: walltime?, overall algo? make sure to not repeat the same as in "algowrap.hpp" here

// lloft objects initialized with an instance of pointcloud
lloft::pointcloud p;
lloft::pointcloud_synth psynth(p);
lloft::pointcloud_io pio(p);

#include <mutex>
std::recursive_mutex m_SubscriberMutex; // ecal callback thread
bool bnewframe = false;

#include "main.h"

#include "init.hpp"
#include "user.hpp"
#include "app_algowrp.hpp"
#include "app_ecal.hpp"

int main(int argc, char** argv)
{
  user::init_Cfg();

  app_ecal::init(argc, argv);

  // (1) load labels from .json file --> this is done only once
  std::string filename;
  filename = argc > 1 ? argv[1] : "C:/GIT/BEV_labeler/labels.json";
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
      // (2) Run the algo,
      //     here: compute z for the given (x,y) using the subscribed ground model --> this is done each frame
      algo::run_with_Mutex();
      auto t2 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> ms_double = t2 - t1;
      std::cout << "run_frs " << ms_double.count() << "ms\n";
      algo_runtime_ms = ms_double.count();
      
      // (3) publish the results
      app_ecal::publish_fox_poly(app_ecal::publisher_poly);

      bnewframe = false;
      _sleep(500); // ms
    }

  } while (!close);

  return 0;
}
