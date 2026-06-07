/*
  Lidar and Perception
  SPS - Scanpattern Synthesizer
  (C) 2024 Automotive ADC GmbH
  author: w.schulz

  Ecal routines: Subscribe, publish
*/

#ifndef APP_ECAL_H
#define APP_ECAL_H

// eCAL
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
// proto i/o
#include "pcl.pb.h"
#include "lane.pb.h"
#include "foxglove/LinePrimitive.pb.h"
#include "foxglove/PointCloud.pb.h"
#include "foxglove/SceneUpdate.pb.h"

// 2do: here or inside algo?
#define FRS_TOPIC  "frs"
#define POLY_TOPIC "bev_anno"
#define FOX_TOPIC  "mop"

namespace app_ecal {
  eCAL::protobuf::CSubscriber<pcl::PointCloud2> subscriber_hrl;
  eCAL::protobuf::CPublisher<pcl::PointCloud2> publisher_frs(FRS_TOPIC);
  eCAL::protobuf::CPublisher<foxglove::SceneUpdate> publisher_poly(POLY_TOPIC);
  eCAL::protobuf::CPublisher<foxglove::PointCloud> publisher_fox(FOX_TOPIC);

  void Pointcloud_Callback(const pcl::PointCloud2 pointcloud_msg);

  void init(int argc, char** argv) {
    // Initialize eCAL
    eCAL::Initialize(argc, argv, "Freespace (eCal)");
    // Create a protobuf subscriber
    // possible topics:
    // - AEyeSensorPointCloudData
    // - gpf_non_ground
    // - meta_pcl
  //  eCAL::protobuf::CSubscriber<pcl::PointCloud2> subscriber_hrl("AEyeSensorPointCloudData");
///    eCAL::protobuf::CSubscriber<pcl::PointCloud2> subscriber_hrl("gse_out");
    subscriber_hrl = eCAL::protobuf::CSubscriber<pcl::PointCloud2>("gse_out");
    // Set the Callback
    subscriber_hrl.AddReceiveCallback(std::bind(&Pointcloud_Callback, std::placeholders::_2));
    // create publisher
    publisher_frs = eCAL::protobuf::CPublisher<pcl::PointCloud2>(FRS_TOPIC);
    publisher_poly = eCAL::protobuf::CPublisher<foxglove::SceneUpdate>(POLY_TOPIC);
    publisher_fox = eCAL::protobuf::CPublisher<foxglove::PointCloud>(FOX_TOPIC);
    // set eCAL state to healthy (--> eCAL Monitor)
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "LANE eCAL publishers initialized");
  }

  void Pointcloud_Callback(const pcl::PointCloud2 pointcloud_msg)
  {
    const std::lock_guard<std::recursive_mutex> lock(m_SubscriberMutex);

    pio.ecal_callback(pointcloud_msg); // 2do: directly give this to AddReceiveCallback
    p.timestamp = (uint64_t)pointcloud_msg.header().stamp().secs() * 10 * 10 * 10 * 10 * 10 * 10 * 10 * 10 * 10 + pointcloud_msg.header().stamp().nsecs();
    p.secs = pointcloud_msg.header().stamp().secs();
    p.nsecs = pointcloud_msg.header().stamp().nsecs();

    memcpy(lloft::vertices, p.pts_raw.data(), p.numpoints * 3 * sizeof(float));
    memcpy(lloft::colors, p.pts_rgb.data(), p.numpoints * 3 * sizeof(float));

    bnewframe = true;
  }

  void publish(eCAL::protobuf::CPublisher<pcl::PointCloud2>& publisher_frs) {
    // Create a protobuf message object
    pcl::PointCloud2 frs_pointcloud;
    std::vector<float> pts;
    for (int i = 0; i < p.pts_raw.size(); i++)
    {
      pts.push_back(p.pts_raw[i].x); pts.push_back(p.pts_raw[i].y); pts.push_back(p.pts_raw[i].z);
      pts.push_back(p.pts_rgb[i].r); pts.push_back(p.pts_rgb[i].g); pts.push_back(p.pts_rgb[i].b);
      pts.push_back(p.pts_class[i]);
    }
    // Create a protobuf message object
    setPointCloud(&frs_pointcloud, { "x","y","z","Processed Intensity","g","b","class" }, pts);// plan: have individual field types
    frs_pointcloud.mutable_header()->mutable_stamp()->set_secs(p.secs);
    frs_pointcloud.mutable_header()->mutable_stamp()->set_nsecs(p.nsecs);
    // Send the message
    publisher_frs.Send(frs_pointcloud);
  }

  void publish_fox_pointcloud(eCAL::protobuf::CPublisher<foxglove::PointCloud>& publisher_gse_fox) {
    foxglove::PointCloud pcl_fox;

    // 2do: even easier: just hand the pointcloud and the serializer will mem-copy itself
    std::vector<float> pts;
    for (int i = 0; i < p.pts_raw.size(); i++)
    {
      pts.push_back(p.pts_raw[i].x); pts.push_back(p.pts_raw[i].y); pts.push_back(p.pts_raw[i].z);
      pts.push_back(p.pts_rgb[i].r); pts.push_back(p.pts_rgb[i].g); pts.push_back(p.pts_rgb[i].b);
      pts.push_back(p.pts_class[i]);
    }
    set_fox_PointCloud(&pcl_fox, { "x","y","z","Processed Intensity","g","b","class" }, pts);// plan: have individual field types

    publisher_gse_fox.Send(pcl_fox);
  }

  // move to frs.cpp/hpp ?
  void publish_Poly(eCAL::protobuf::CPublisher<foxglove::LinePrimitive>& publisher_poly) {
/*    foxglove::LinePrimitive poly;
    for (int i = 0; i < p_freespace->nPoly; i++) {
      poly.mutable_points()->Add();
      int max_idx = poly.mutable_points()->size() - 1;
      poly.mutable_points(max_idx)->set_x(p_freespace->a_Poly[i].x);
      poly.mutable_points(max_idx)->set_y(p_freespace->a_Poly[i].y);
      poly.mutable_indices()->Add(i);
    }
    publisher_poly.Send(poly);
*/
  }

  void publish_fox_poly(eCAL::protobuf::CPublisher<foxglove::SceneUpdate>& publisher_foxScene) {
    foxglove::SceneUpdate scu;
    foxglove::SceneEntity* sce;
    foxglove::LinePrimitive* poly;

    sce = scu.mutable_entities()->Add();
//    for (int i = 0; i < algo::p_freespace->nPoly; i++) {
    for (const auto& ann : algo::annotations) {
      poly = sce->mutable_lines()->Add();
      for (const auto& p : ann.points) {
        poly->mutable_points()->Add();
        int max_idx = poly->mutable_points()->size() - 1;
        poly->mutable_points(max_idx)->set_x(p.x);
        poly->mutable_points(max_idx)->set_y(p.y);
        poly->mutable_points(max_idx)->set_z(-2.7f); // 2do: use ground model
        // set colors in Foxglove / Lichtblick
      }
      poly->set_thickness(.3f);
      poly->set_type(foxglove::LinePrimitive_Type_LINE_STRIP);
    }
//    sce->set_frame_id(p.frame_id); // frame_id from input pointcloud, if any operation changes the frame, this has to be adapted
    sce->set_frame_id("lidar"); // frame_id from input pointcloud, if any operation changes the frame, this has to be adapted
    publisher_foxScene.Send(scu);
  }


}

#endif// APP_ECAL_H
