#ifndef APP_ECAL_H
#define APP_ECAL_H

// eCAL
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
// proto i/o
#include "pcl.pb.h"
#include "gsm.pb.h" // 2do: simplify, i.e. only poly coefficients, check foxglove for suitable message type
#include "foxglove/LinePrimitive.pb.h"
#include "foxglove/PointCloud.pb.h"
#include "foxglove/SceneUpdate.pb.h"


// Topics "In"
#define TOPIC_IN_POINTCLOUD "gse_out"
#define TOPIC_IN_GMODEL     "g_model"
// Topics "Out"
#define TOPIC_OUT_POLY      "bev_anno"
#define INHERIT_FRAME_ID    false  // otherwise use FRAME_ID_OUT, 2do: make variable
#define FRAME_ID_OUT        "lidar"


namespace app_ecal {
  eCAL::protobuf::CSubscriber<pcl::PointCloud2> subscriber_hrl; // pointcloud - actually not needed
  eCAL::protobuf::CSubscriber<gsm::GSM_t_ProPortList> subscriber_gsm; // ground model
  eCAL::protobuf::CPublisher<foxglove::SceneUpdate> publisher_poly(TOPIC_OUT_POLY);

  void Pointcloud_Callback(const pcl::PointCloud2 pointcloud_msg);
  void GModel_callback(const gsm::GSM_t_ProPortList msg);

  void init(int argc, char** argv) {
    // Initialize eCAL
    eCAL::Initialize(argc, argv, "BEV_Anno");
    // Create protobuf subscriber(s)
    subscriber_hrl = eCAL::protobuf::CSubscriber<pcl::PointCloud2>(TOPIC_IN_POINTCLOUD);
    subscriber_gsm = eCAL::protobuf::CSubscriber<gsm::GSM_t_ProPortList>(TOPIC_IN_GMODEL); // 2do: simplify, i.e. only poly coefficients,
                                                                                           //      no composite publisher
                                                                                           //      however this involves changes in GSE!!
    // Set the Callback
    subscriber_hrl.AddReceiveCallback(std::bind(&Pointcloud_Callback, std::placeholders::_2));
    subscriber_gsm.AddReceiveCallback(std::bind(&GModel_callback, std::placeholders::_2));
    // Create publisher(s)
    publisher_poly = eCAL::protobuf::CPublisher<foxglove::SceneUpdate>(TOPIC_OUT_POLY);
    // Set eCAL state to healthy (--> eCAL Monitor)
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

  void GModel_callback(const gsm::GSM_t_ProPortList msg) {
    // 2do: subscribe to the ground model, i.e. plane or quadratic coefficients,
    // and save them in algo:: namespace for use in algo::calc_ground_pt()
    const std::lock_guard<std::recursive_mutex> lock(m_SubscriberMutex);
//    msg.GroundModel().CopyTo(&algo::gmodel);
    algo::gmodel.m_a1 = msg.groundmodelparams().modelx();
    algo::gmodel.m_a2 = msg.groundmodelparams().modelx2();
    algo::gmodel.m_b1 = msg.groundmodelparams().modely();
    algo::gmodel.m_b2 = msg.groundmodelparams().modely2();
    algo::gmodel.m_d  = msg.groundmodelparams().modeld();

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
//        poly->mutable_points(max_idx)->set_z(-2.7f); // hack! some value that fits for Truck
        float z = algo::QuadraticPlaneModel(p.x, p.y);
        poly->mutable_points(max_idx)->set_z(z); // using the ground model
        // set colors in Foxglove / Lichtblick
      }
      poly->set_thickness(.3f);
      poly->set_type(foxglove::LinePrimitive_Type_LINE_STRIP);
    }
#if INHERIT_FRAME_ID
    sce->set_frame_id(p.frame_id);   // frame_id from input pointcloud, if any operation changes the frame, this has to be adapted
#else
    sce->set_frame_id(FRAME_ID_OUT); // adapting ;-)
#endif
    publisher_foxScene.Send(scu);
  }


}

#endif// APP_ECAL_H
