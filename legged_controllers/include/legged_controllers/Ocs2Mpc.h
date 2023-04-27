//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>

#include "ocs2_msgs/mpc_observation.h"
#include "wolf_msgs/Wrench.h"
#include "wolf_msgs/Cartesian.h"
#include "wolf_msgs/Postural.h"
namespace legged {

using namespace ocs2;
using namespace legged_robot;

class MpcClass {
 public:
  MpcClass() = default;
  ~MpcClass(){
    std::cerr << "########################################################################";
    std::cerr << "\n### MPC Benchmarking";
    std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
    std::cerr << "########################################################################";
  }
  bool init(ros::NodeHandle& controller_nh);
  void update();
  void starting();
  void stopping(const ros::Time& /*time*/) { mpcRunning_ = false; }

 protected:

  void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                            bool verbose);
  void setupMpc();
  void setupMrt();
  // Interface
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;

  // State Estimation
  SystemObservation currentObservation_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // MPC Output
  ros::Publisher mpcWrenchPublisher1_;
  ros::Publisher mpcFootPublisher1_;
  ros::Publisher mpcWrenchPublisher2_;
  ros::Publisher mpcFootPublisher2_;
  ros::Publisher mpcWrenchPublisher3_;
  ros::Publisher mpcFootPublisher3_;
  ros::Publisher mpcWrenchPublisher4_;
  ros::Publisher mpcFootPublisher4_;
  ros::Publisher mpcBasePublisher_;
  ros::Publisher mpcPosturalPublisher_;

  // Observation Input
  ros::Subscriber mpcObservation_;
  void observationCallback(const ocs2_msgs::mpc_observationConstPtr& msg);

 private:
  std::atomic_bool mpcRunning_{};
  benchmark::RepeatedTimer mpcTimer_;
};

} // namespace legged
