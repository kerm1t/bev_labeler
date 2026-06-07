#ifndef POINTCLOUD_SYNTH_H
#define POINTCLOUD_SYNTH_H

#include "include/pointcloud/pointcloud.hpp"

#include <iostream>

namespace lloft {
  /*  purpose:  fill point cloud with synthetic data
      date:     5/20/2023
      author:   w.schulz
        */
  
  class pointcloud_synth : public pointbase {
  public:
    pointcloud& p;
    pointcloud_synth(pointcloud& pc) : p(pc) {
    }
    
    void random(int numpoints);
    void test_upscale(int nrows, int ncols, float dx);
  };

  void pointcloud_synth::test_upscale(int nrows, int ncols, float dx)
  {
    //  int nrows = 5;
    //  int ncols = 10;
///    memset(pointcloud_ordered, nrows * ncols * sizeof(point_ord), 0);
    for (int j = 0; j < nrows; j++) {
      for (int k = 0; k < ncols; k++) {
        point pt;
        pt.x = dx;                 // distance
        pt.y = k * 10.0f;          // cols
        pt.z = 1.0f + j * 1.0f;    // rows
        p.pts_raw.push_back(pt);
        point_rgb ptcol;
        ptcol.r = 0;
        ptcol.g = 0;
        ptcol.b = 0;
        p.pts_rgb.push_back(ptcol);
        // cartesian to spherical : calc az,el,d
/*        float d = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        float az = atan2(pt.y, pt.x);
        float el = atan2(pt.z, sqrt(pt.x * pt.x + pt.y * pt.y));

        pointcloud_ordered[j][k].x = pt.x;
        pointcloud_ordered[j][k].y = pt.y;
        pointcloud_ordered[j][k].z = pt.z;
        pointcloud_ordered[j][k].az = az * 10000.0f;
        pointcloud_ordered[j][k].el = el * 10000.0f;
        pointcloud_ordered[j][k].d = d;
  */    }
    }
    p.numpoints = p.pts_raw.size();
  }

  // a) fill with random data
  void pointcloud_synth::random(int numpoints)
  {
//    p.bbox = (0.0f, 0.0f, 0.0f, 0.0f);
    srand(time(NULL));
    for (int i = 0; i < numpoints; i++)
    {
      point pt;
      pt.x = -7.5f + (int)(rand() % 1000) / 1000.0f * 15.0f;
      pt.y = -7.5f + (int)(rand() % 1000) / 1000.0f * 15.0f;
      pt.z = (int)(rand() % 1000) / 1000.0f * 5.0f;
      p.pts_raw.push_back(pt);

      point_rgb ptcol;
      ptcol.r = rand() % 200;
      ptcol.g = rand() % 200;
      ptcol.b = rand() % 200;
      p.pts_rgb.push_back(ptcol);
//      pt.r = 99;
//      pt.g = 99;B
//      pt.b = 99;
//      m_measPoints[1].push_back(pt);
    }
    p.numpoints = p.pts_raw.size();
  }


}
#endif POINTCLOUD_IO_H
