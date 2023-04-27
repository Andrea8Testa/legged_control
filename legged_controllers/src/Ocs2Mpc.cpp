//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_controllers/Ocs2Mpc.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged {

bool MpcClass::init(ros::NodeHandle& mpc_nh) {
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  mpc_nh.getParam("/urdfFile", urdfFile);
  mpc_nh.getParam("/taskFile", taskFile);
  mpc_nh.getParam("/referenceFile", referenceFile);
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();

  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);

  // MPC publishers
  mpcWrenchPublisher1_ = nh.advertise<wolf_msgs::Force>("mpc_wrench_lf", 1);
  mpcFootPublisher1_ = nh.advertise<wolf_msgs::Cartesian>("mpc_foot_lf", 1);
  mpcWrenchPublisher2_ = nh.advertise<wolf_msgs::Force>("mpc_wrench_lh", 1);
  mpcFootPublisher2_ = nh.advertise<wolf_msgs::Cartesian>("mpc_foot_lh", 1);
  mpcWrenchPublisher3_ = nh.advertise<wolf_msgs::Force>("mpc_wrench_rf", 1);
  mpcFootPublisher3_ = nh.advertise<wolf_msgs::Cartesian>("mpc_foot_rf", 1);
  mpcWrenchPublisher4_ = nh.advertise<wolf_msgs::Force>("mpc_wrench_rh", 1);
  mpcFootPublisher4_ = nh.advertise<wolf_msgs::Cartesian>("mpc_foot_rh", 1);
  mpcBasePublisher_ = nh.advertise<wolf_msgs::Cartesian>("mpc_base", 1);
  mpcPosturalPublisher_ = nh.advertise<wolf_msgs::Postural>("mpc_postural", 1);

  return true;
}

void MpcClass::starting() {
  // Initial state
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  currentObservation_.time += ros::Duration(0.002).toSec();
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpcRunning_ = true;
}

void MpcClass::update() {

  try {
    executeAndSleep(
        [&]() {
          if (mpcRunning_) {
            mpcTimer_.startTimer();
            mpcMrtInterface_->advanceMpc();
            mpcTimer_.endTimer();
          }
        },
        leggedInterface_->mpcSettings().mpcDesiredFrequency_);
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
  }
}

void MpcClass::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose) {
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void MpcClass::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());

  const std::string robotName = "legged_robot";
  ros::NodeHandle nh;
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
}

void MpcClass::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();
}

void MpcClass::observationCallback(const ocs2_msgs::mpc_observationConstPtr& msg){

  currentObservation_.time = msg->time;
  currentObservation_.state.setZero();
  currentObservation_.input.setZero();
  // Not working
  currentObservation_.state.lazyAssign(msg->state.value);
  currentObservation_.input.lazyAssign(msg->input.value);
  currentObservation_.mode = msg->mode;

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

  // Retrieve MPC ptimized output
  vector_t mpc_posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  vector_t mpc_velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());
  vector_t mpc_basePosDes_eul = centroidal_model::getBasePose(optimizedState, leggedInterface_->getCentroidalModelInfo());

  // Conversion from ZYX euler to quaternion
  // Abbreviations for the various angular functions
  double cr = cos(mpc_basePosDes_eul(5) * 0.5);
  double sr = sin(mpc_basePosDes_eul(5) * 0.5);
  double cp = cos(mpc_basePosDes_eul(4) * 0.5);
  double sp = sin(mpc_basePosDes_eul(4) * 0.5);
  double cy = cos(mpc_basePosDes_eul(3) * 0.5);
  double sy = sin(mpc_basePosDes_eul(3) * 0.5);

  Eigen::Quaterniond mpc_base_quat;
  mpc_base_quat.w() = cr * cp * cy + sr * sp * sy;
  mpc_base_quat.x() = sr * cp * cy - cr * sp * sy;
  mpc_base_quat.y() = cr * sp * cy + sr * cp * sy;
  mpc_base_quat.z() = cr * cp * sy - sr * sp * cy;

//  std::vector<size_t> contactIds = leggedInterface_->getCentroidalModelInfo().endEffectorFrameIndices;
  // Absolute ids not required. Ids are referred to leggedInterface_->getCentroidalModelInfo().numThreeDofContacts
  vector_t mpc_contactDes1 = centroidal_model::getContactForces(optimizedInput, 0, leggedInterface_->getCentroidalModelInfo());
  vector_t mpc_contactDes2 = centroidal_model::getContactForces(optimizedInput, 1, leggedInterface_->getCentroidalModelInfo());
  vector_t mpc_contactDes3 = centroidal_model::getContactForces(optimizedInput, 2, leggedInterface_->getCentroidalModelInfo());
  vector_t mpc_contactDes4 = centroidal_model::getContactForces(optimizedInput, 3, leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());
  const auto& mpc_model = leggedInterface_->getPinocchioInterface().getModel();
  auto& mpc_data = leggedInterface_->getPinocchioInterface().getData();
  pinocchio::forwardKinematics(mpc_model, mpc_data, centroidal_model::getGeneralizedCoordinates(optimizedState, leggedInterface_->getCentroidalModelInfo()));
  pinocchio::updateFramePlacements(mpc_model, mpc_data);
  std::vector<vector3_t> mpc_foot_pos = eeKinematicsPtr_->getPosition(optimizedState);
  std::vector<vector3_t> mpc_foot_vel = eeKinematicsPtr_->getVelocity(optimizedState, optimizedInput);

  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  pinocchioMapping.setPinocchioInterface(leggedInterface_->getPinocchioInterface());
  const auto qDesired = pinocchioMapping.getPinocchioJointPosition(optimizedState);
  ocs2::updateCentroidalDynamics(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(), qDesired);
  const vector_t vDesired = pinocchioMapping.getPinocchioJointVelocity(optimizedState, optimizedInput);

  // Pack messages
  wolf_msgs::Force force_msg_1, force_msg_2, force_msg_3, force_msg_4;
  force_msg_1.force.force.x = mpc_contactDes1(0);
  force_msg_1.force.force.y = mpc_contactDes1(1);
  force_msg_1.force.force.z = mpc_contactDes1(2);
  force_msg_2.force.force.x = mpc_contactDes2(0);
  force_msg_2.force.force.y = mpc_contactDes2(1);
  force_msg_2.force.force.z = mpc_contactDes2(2);
  force_msg_3.force.force.x = mpc_contactDes3(0);
  force_msg_3.force.force.y = mpc_contactDes3(1);
  force_msg_3.force.force.z = mpc_contactDes3(2);
  force_msg_4.force.force.x = mpc_contactDes4(0);
  force_msg_4.force.force.y = mpc_contactDes4(1);
  force_msg_4.force.force.z = mpc_contactDes4(2);

  wolf_msgs::Cartesian foot_msg_1, foot_msg_2, foot_msg_3, foot_msg_4;
  foot_msg_1.pose.position.x = mpc_foot_pos[0](0);
  foot_msg_1.pose.position.y = mpc_foot_pos[0](1);
  foot_msg_1.pose.position.z = mpc_foot_pos[0](2);
  foot_msg_1.twist.linear.x = mpc_foot_vel[0](1);
  foot_msg_1.twist.linear.y = mpc_foot_vel[0](2);
  foot_msg_1.twist.linear.z = mpc_foot_vel[0](3);
  foot_msg_2.pose.position.x = mpc_foot_pos[1](0);
  foot_msg_2.pose.position.y = mpc_foot_pos[1](1);
  foot_msg_2.pose.position.z = mpc_foot_pos[1](2);
  foot_msg_2.twist.linear.x = mpc_foot_vel[1](1);
  foot_msg_2.twist.linear.y = mpc_foot_vel[1](2);
  foot_msg_2.twist.linear.z = mpc_foot_vel[1](3);
  foot_msg_3.pose.position.x = mpc_foot_pos[2](0);
  foot_msg_3.pose.position.y = mpc_foot_pos[2](1);
  foot_msg_3.pose.position.z = mpc_foot_pos[2](2);
  foot_msg_3.twist.linear.x = mpc_foot_vel[2](1);
  foot_msg_3.twist.linear.y = mpc_foot_vel[2](2);
  foot_msg_3.twist.linear.z = mpc_foot_vel[2](3);
  foot_msg_4.pose.position.x = mpc_foot_pos[3](0);
  foot_msg_4.pose.position.y = mpc_foot_pos[3](1);
  foot_msg_4.pose.position.z = mpc_foot_pos[3](2);
  foot_msg_4.twist.linear.x = mpc_foot_vel[3](1);
  foot_msg_4.twist.linear.y = mpc_foot_vel[3](2);
  foot_msg_4.twist.linear.z = mpc_foot_vel[3](3);

  wolf_msgs::Cartesian base_msg;
  base_msg.pose.position.x = mpc_basePosDes_eul(0);
  base_msg.pose.position.y = mpc_basePosDes_eul(1);
  base_msg.pose.position.z = mpc_basePosDes_eul(2);
  base_msg.pose.orientation.w = mpc_base_quat.w();
  base_msg.pose.orientation.x = mpc_base_quat.x();
  base_msg.pose.orientation.y = mpc_base_quat.y();
  base_msg.pose.orientation.z = mpc_base_quat.z();

  base_msg.twist.linear.x = vDesired(0);
  base_msg.twist.linear.y = vDesired(1);
  base_msg.twist.linear.z = vDesired(2);
  base_msg.twist.angular.z = vDesired(3);
  base_msg.twist.angular.y = vDesired(4);
  base_msg.twist.angular.x = vDesired(5);

  wolf_msgs::Postural postural_msg;
  for (size_t i = 0; i < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++i) {
    postural_msg.positions.push_back(mpc_posDes(i));
    postural_msg.velocities.push_back(mpc_velDes(i));
  }

  // Publish the MPC output
  mpcWrenchPublisher1_.publish(force_msg_1);
  mpcFootPublisher1_.publish(foot_msg_1);
  mpcWrenchPublisher2_.publish(force_msg_2);
  mpcFootPublisher2_.publish(foot_msg_2);
  mpcWrenchPublisher3_.publish(force_msg_3);
  mpcFootPublisher3_.publish(foot_msg_3);
  mpcWrenchPublisher4_.publish(force_msg_4);
  mpcFootPublisher4_.publish(foot_msg_4);
  mpcBasePublisher_.publish(base_msg);
  mpcPosturalPublisher_.publish(postural_msg);
}

} // namespace legged

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ocs2_mpc");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);

  legged::MpcClass ocs2_mpc;

  ocs2_mpc.init(nh);
  ocs2_mpc.starting();

  while (ros::ok())
  {
    ocs2_mpc.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
