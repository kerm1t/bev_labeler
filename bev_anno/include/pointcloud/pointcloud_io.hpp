#ifndef POINTCLOUD_IO_H
#define POINTCLOUD_IO_H

#include "include/pointcloud/pointcloud.hpp"
#include "include/util/color.h"

#include <string>
#include <vector>

#define ECAL true
#ifdef ECAL
// proto
#include "pcl.pb.h"
// unpacker for Pointcloud2
#include "../if/hndl_pointcloud.h"
#endif

namespace lloft {
  /*  purpose:  point cloud input / output
      date:     5/20/2023
      author:   w.schulz
        */
  bool publish_ground;

  enum io_type { ascii, binary };

  class pointcloud_io : public pointcloud {
  public:
    pointcloud& p;
    pointcloud_io(pointcloud& pc) : p(pc) {
    }
#ifdef ECAL
    void ecal_callback(const pcl::PointCloud2& pc);
#endif
    void load_pcd(std::string filename, io_type iot);
    void load_ply();
    void write_pcd(std::string filename, io_type iot);
    void write_ply();
  };

#ifdef ECAL
  // Callback for eCal message -> fill pointcloud
  void pointcloud_io::ecal_callback(const pcl::PointCloud2& pc)
  {
    //  ecalmsg = pointcloud_msg.numpoints();
    std::vector<double> pcl2_TMP_points;
    std::vector<std::string> fields = {
      "x", "y", "z",
///      "intensity", // case insensitive ;-)
///      "Processed Intensity",
      "class",
      "hog" // height over ground
    };
    getPointFields(pc, pcl2_TMP_points, fields);
    if (pcl2_TMP_points.size() == 0) return; // most probably some field name spelled wrongly

    numpoints = pc.width();
    int pointCount = numpoints;
    int pointSize = pc.point_step();
    frameid = atoi(pc.header().frame_id().c_str());
    
    // process
    p.pts_raw.clear();
    p.pts_rgb.clear();
    p.pts_class.clear();
    p.pts_hog.clear(); // height over ground (from GSE) 

    // OpenGL
    float r, g, b;

    // (iv) visualize point cloud
    for (int i = 0; i < numpoints; i++)
    {
      float pt_class = pcl2_TMP_points[i * fields.size() + 3];
      float pt_hog = pcl2_TMP_points[i * fields.size() + 4];
      if (!publish_ground && pt_class < 0.5) continue; // 0.0=ground, 0.5=non-ground, 0.8=within roadboundary
      // avoid ground to limit freespace / road border
// need to move to freespace Fan
//      if (pt_hog < 0.5) continue;
//      if (pt_hog > 3.0) continue;

      point pt;
      pt.x = pcl2_TMP_points[i * fields.size()];
      pt.y = pcl2_TMP_points[i * fields.size() + 1];
      pt.z = pcl2_TMP_points[i * fields.size() + 2];

      point_rgb ptcol;
      ptcol.r = 0.3;// b * 255;
      ptcol.g = .3;// r * 255;
      ptcol.b = .3;// g * 255;

      p.pts_raw.push_back(pt);
      p.pts_rgb.push_back(ptcol);
      p.pts_class.push_back(pt_class);
      p.pts_hog.push_back(pt_hog);
    }
    p.numpoints = p.pts_raw.size();
  }
#endif // ECAL

}
#endif // POINTCLOUD_IO_H
