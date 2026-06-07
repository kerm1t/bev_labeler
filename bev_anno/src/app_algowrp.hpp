#ifndef APP_ALGOWRP_HPP
#define APP_ALGOWRP_HPP

#include <chrono>

namespace algo {

  float runtime_ms;

  void run_with_Mutex() {
    const std::lock_guard<std::recursive_mutex> lock(m_SubscriberMutex);
    auto t1 = std::chrono::high_resolution_clock::now();

// do something

    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> ms_double = t2 - t1;
    std::cout << "run_bev_anno " << ms_double.count() << "ms\n";
    runtime_ms = ms_double.count();
  }
}

#endif APP_ALGOWRP_HPP