#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "include/math/linmath.h"
#include "include/math/mathbase.h"

#include <memory.h>
#include <vector>
#include <atomic>

namespace lloft {
  // semantic
  // zenseact

  // OpenGL
  //std::vector<float> vertices;
  //std::vector<float> colors; // height, dist, intensity
  constexpr auto POINTCL_MAXPOINTS = 700000;// <-- Hesai 50000//20000;
  static float vertices[POINTCL_MAXPOINTS * 3];
  static float colors[POINTCL_MAXPOINTS * 3];
  static int nvertices;
/// tut's auch nicht  std::atomic<int> nvertices;

  struct point {
    float x;
    float y;
    float z;
  };
  typedef std::vector<point> points_raw;
/*    int inten;
    int az;   // RAD/10000
    int el;   // RAD/10000
    float d;
*/
  // the following vector has the same size as "points_raw" vector, it will be given to GPU
  struct point_rgb {
    float r;// unsigned char r;
    float g; // need to be same as gpu mem, to do memcpy
    float b;
  };
  typedef std::vector<point_rgb> points_rgb;

  enum point_class { road, below_road, above_road, lane_marker };
  struct point_sem {
    float x;
    float y;
    float z;
    int inten;
    int az;   // RAD/10000
    int el;   // RAD/10000
    float d;
    point_class pclass;
  };
  typedef std::vector<point> points_semantic;

  enum color_coding { cc_height, cc_distance }; // --> to visualization ?
  static color_coding colorcoding = cc_height;
  static int intensityclamp[2] = { 0, 10000 };

/* HRL specific
  ===============
  max col + row differ for scan patterns, e.g.
  
  Berlin 2492 x 1080,
  VW          x 810,
  Volvo  1700 x 1070

  however not all opf these col and rows are used
  also the acquisition is rather by oversampling a line.
  so point cloud is unordered (column wise) 
  next up: order/sort the unordered hrl point cloud
*/
#define MAXCOL 2500
#define MAXROW 1200

#define MAXUSEDROW 80
#define MAXUSEDCOL 1700

  static int points_set[MAXROW + 1][MAXCOL + 1];
  static int rows_set[MAXROW + 1];
  static int cols_set[MAXCOL + 1];
  static int image[MAXUSEDROW][MAXUSEDCOL];
  // whoooo big data coming
/*  struct point_ord {
    float x;
    float y;
    float z;
    float d;
    float az; // <- int
    float el; // <- int
    int inten;
  };
  point_ord points_ordered[MAXUSEDROW][MAXUSEDCOL]; 
  */

  class pointbase {
  };
  
  class pointcloud : public pointbase {
  public:
    int frameid;
    uint64_t timestamp;
    uint32_t secs, nsecs;
    int numpoints;
    bool ordered;
    int numrows;
    int numcols;
    points_raw pts_raw;
    std::vector<float> pts_class; // float, for the sake of simplicity setting up a pc2
    std::vector<float> pts_hog;
    points_rgb pts_rgb;
    points_raw* pts_ord; // optional

    int size(); // = numpoints = width * height
    float loadtime();
    vec4 bbox;
  };
  
  class pointcloud_HRL : public pointcloud {
    void sort();
    // to_pointcloud2()
  };

  int pointcloud::size() {
    return this->numpoints;
  }

  float pointcloud::loadtime() {
    return 0; // replace dummy
  }

}
#endif // POINTCLOUD_H
