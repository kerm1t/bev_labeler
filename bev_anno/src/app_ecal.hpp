#ifndef APP_ECAL_H
#define APP_ECAL_H

// eCAL
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
// proto i/o
#include "pcl.pb.h"
#include "foxglove/LinePrimitive.pb.h"
#include "foxglove/PointCloud.pb.h"
#include "foxglove/SceneUpdate.pb.h"

// 2do: here or inside algo?
#define POLY_TOPIC "bev_anno"

namespace app_ecal {
  eCAL::protobuf::CSubscriber<pcl::PointCloud2> subscriber_hrl;
  eCAL::protobuf::CPublisher<foxglove::SceneUpdate> publisher_poly(POLY_TOPIC);

  void Pointcloud_Callback(const pcl::PointCloud2 pointcloud_msg);

  void init(int argc, char** argv) {
    // Initialize eCAL
    eCAL::Initialize(argc, argv, "BEV_Anno");
    // Create a protobuf subscriber
    subscriber_hrl = eCAL::protobuf::CSubscriber<pcl::PointCloud2>("gse_out");
    // Set the Callback
    subscriber_hrl.AddReceiveCallback(std::bind(&Pointcloud_Callback, std::placeholders::_2));
    // create publisher
    publisher_poly = eCAL::protobuf::CPublisher<foxglove::SceneUpdate>(POLY_TOPIC);
    // set eCAL state to healthy (--> eCAL Monitor)
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "BEV_Anno eCAL publishers initialized");
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

  void publish_fox_poly(eCAL::protobuf::CPublisher<foxglove::SceneUpdate>& publisher_foxScene) {
    foxglove::SceneUpdate scu;
    foxglove::SceneEntity* sce;
    foxglove::LinePrimitive* poly;

    sce = scu.mutable_entities()->Add();
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
