#include "linmath.h"
#include <memory.h>
#include "pointcloud.hpp"

namespace lloft {
  // semantic
  // zenseact

  class pointcloud_op : public pointcloud {
    void organize(); // prestep to order/sort
    void order_to_image();
    void order(); // == sort()
    void upscale();
    void upscale_cart();
    void filter(); // s. META
    void rotate(vec3 r);
    void translate(vec3 t);
  };
  
  class pointcloud_HRL : public pointcloud {
    void sort();
    // to_pointcloud2()
  };

  void pointcloud_op::organize() {
    memset(points_set, 0, MAXCOL * MAXROW);
    memset(cols_set, 0, MAXCOL + 1);
    memset(rows_set, 0, MAXROW + 1);

    //  for (int i = 0; i < maxrow; i++) rowlookup[j++] = i;
    //  for (int i = 0; i < maxcol; i++) collookup[j++] = i;
      // (B) set order structs
    int maxusedcol = 0;
    int maxusedrow = 0;
    for (int i = 0; i < numpoints-1; i++)
    {
      int col = points[i * fields.size() + 5];
      int row = points[i * fields.size() + 6];
      if (col > maxusedcol) maxusedcol = col;
      if (row > maxusedrow) maxusedrow = row;
      points_set[row][col] = i;
      cols_set[col]++;
      rows_set[row]++;
    }
    int ncols = 0;
    int nrows = 0;
    for (int i = 0; i <= maxusedcol; i++) if (cols_set[i] > 0) ncols++;
    for (int i = 0; i <= maxusedrow; i++) if (rows_set[i] > 0) nrows++;
    int rowlookup[MAXUSEDROW + 1];
    int collookup[MAXUSEDCOL + 1];
    int j = 0;
    for (int i = 0; i <= maxusedcol; i++) if (cols_set[i] > 0) collookup[j++] = i;
    j = 0;
    for (int i = 0; i <= maxusedrow; i++) if (rows_set[i] > 0) rowlookup[j++] = i;
  }

  void pointcloud_op::order_to_image() {
    for (int i = 0; i < numpoints-1; i++)
    {
      int col = points[i * fields.size() + 5];
      int row = points[i * fields.size() + 6];
      bool bbreak = false;
      for (int j = 0; j < MAXUSEDROW; j++) {
        if (row == rowlookup[j]) {
          for (int k = 0; k < MAXUSEDCOL; k++) {
            if (col == collookup[k]) {
              image[j][k] = points[i * fields.size() + 4]; // intensity
              //            image[j][k] = points[i * fields.size() + 9]; // radius
              bbreak = true;
              break;
            }
          }
          if (bbreak) break;
        }
      }
    }
  }

  void pointcloud_op::order() {
    memset(pointcloud_ordered, 0, MAXUSEDROW * MAXUSEDCOL * sizeof(point_ord));
    for (int i = 0; i < ecal_numpoints - 1; i++)
    {
      int col = points[i * fields.size() + 5];
      int row = points[i * fields.size() + 6];
      bool bbreak = false;
      for (int j = 0; j < nrows; j++) {
        if (row == rowlookup[j]) {
          for (int k = 0; k < ncols; k++) {
            if (col == collookup[k]) {
              pointcloud_ordered[j][k].x = points[i * fields.size()]; // x
              pointcloud_ordered[j][k].y = points[i * fields.size() + 1]; // y
              pointcloud_ordered[j][k].z = points[i * fields.size() + 2]; // z
              pointcloud_ordered[j][k].inten = points[i * fields.size() + 4]; // intensity
              pointcloud_ordered[j][k].d = points[i * fields.size() + 9]; // radius
              pointcloud_ordered[j][k].az = points[i * fields.size() + 8]; // azimuth
              pointcloud_ordered[j][k].el = points[i * fields.size() + 10]; // elevation
              bbreak = true;
              break;
            }
          }
          if (bbreak) break;
        }
      }
    }
  }

//  void pointcloud_op::upscale(int nrows, int ncols) {
  void pointcloud_op::upscale() {
      // add points inbetween
    for (int j = 0; j < nrows; j++) {
      for (int k = 0; k < ncols - 1; k++) {
        MeasurementPoint3D pt;
        // (a) one point to the next col
        float el = pointcloud_ordered[j][k].el;
        float az = lerp(pointcloud_ordered[j][k].az, pointcloud_ordered[j][k + 1].az, 0.5f);
        float d = lerp(pointcloud_ordered[j][k].d, pointcloud_ordered[j][k + 1].d, 0.5f);
        // sin az = x / d
        // sin el = y / d
        // cos az = z / d
  //#define DEGTORAD(x) x*3.14159f/180.0f
#define RADCONV(x) x/10000.0f
        if (el > 0 || az > 0)
        {
          float x = cos(RADCONV(az)) * d;
          float y = sin(RADCONV(az)) * d;
          float z = sin(RADCONV(el)) * d;
          pt.x = x;
          pt.y = y;
          pt.z = z;
          pt.r = 255;
          pt.g = 255;
          pt.b = 255;
          m_measPoints[0].push_back(pt);
        }
      }
    }
    for (int j = 0; j < nrows - 1; j++) {
      for (int k = 0; k < ncols; k++) {
        MeasurementPoint3D pt;
        // (b) one point to the next row
        float el = lerp(pointcloud_ordered[j][k].el, pointcloud_ordered[j + 1][k].el, 0.5f);
        float az = pointcloud_ordered[j][k].az;
        float d = lerp(pointcloud_ordered[j][k].d, pointcloud_ordered[j + 1][k].d, 0.5f);
        // sin az = x / d
        // sin el = y / d
        // cos az = z / d
        if (el > 0 || az > 0)
        {
          float x = cos(RADCONV(az)) * d;
          float y = sin(RADCONV(az)) * d;
          float z = sin(RADCONV(el)) * d;
          pt.x = x;
          pt.y = y;
          pt.z = z;
          pt.r = 255;
          pt.g = 255;
          pt.b = 255;
          m_measPoints[0].push_back(pt);
        }
      }
    }
  }

//  void upscale_cart(int nrows, int ncols) {
  void pointcloud_op::upscale_cart() {
    // add points inbetween
    for (int j = 0; j < nrows; j++) {
      for (int k = 0; k < ncols - 1; k++) {
        MeasurementPoint3D pt;
        // (a) one point to the next col
        Vec3f a = Vec3f(pointcloud_ordered[j][k].x, pointcloud_ordered[j][k].y, pointcloud_ordered[j][k].z);
        Vec3f b = Vec3f(pointcloud_ordered[j][k + 1].x, pointcloud_ordered[j][k + 1].y, pointcloud_ordered[j][k].z);
        Vec3f i = lerp2(a, b, 0.5f);
        pt.x = i.x;
        pt.y = i.y;
        pt.z = i.z;
        pt.r = 255;
        pt.g = 255;
        pt.b = 255;
        m_measPoints[0].push_back(pt);
      }
    }
    /*for (int j = 0; j < nrows - 1; j++) {
      for (int k = 0; k < ncols; k++) {
        MeasurementPoint3D pt;
        // (b) one point to the next row
        Vec3f a = Vec3f(pointcloud_ordered[j][k].x, pointcloud_ordered[j][k].y, pointcloud_ordered[j][k].z);
        Vec3f b = Vec3f(pointcloud_ordered[j+1][k].x, pointcloud_ordered[j+1][k].y, pointcloud_ordered[j+1][k].z);
        Vec3f i = lerp2(a, b, 0.5f);
        pt.x = i.x;
        pt.y = i.y;
        pt.z = i.z;
        pt.r = 255;
        pt.g = 255;
        pt.b = 255;
        m_measPoints[0].push_back(pt);
      }
    }*/
  }


}
