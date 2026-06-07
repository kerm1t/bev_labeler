#ifndef APP_ALGOWRP_HPP
#define APP_ALGOWRP_HPP

#include <chrono>

namespace algo {
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

  float runtime_ms;
  auto annotations = std::vector<Annotation>();

  void print_labels(const std::vector<Annotation>& annotations) {
    for (const auto& ann : annotations) {
      std::cout << "[" << ann.cls << "]"
        << (ann.closed ? " (closed)" : " (open)")
        << "  " << ann.points.size() << " points\n";
      for (const auto& p : ann.points)
        std::cout << "  (" << p.x << ", " << p.y << ")\n";
    }
  }

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