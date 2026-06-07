#ifndef ANNO_IO_H
#define ANNO_IO_H

// read annotations from bev labeler
// 2do:
// - what is the most common label format? kitty, nuScenes?

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

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
} // namespace algo

#endif // ANNO_IO_H
