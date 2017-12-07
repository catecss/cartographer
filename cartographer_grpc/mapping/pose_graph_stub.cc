/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_grpc/mapping/pose_graph_stub.h"

#include "glog/logging.h"

namespace cartographer_grpc {
namespace mapping {

PoseGraphStub::PoseGraphStub(std::shared_ptr<grpc::Channel> client_channel)
    : client_channel_(client_channel) {
  LOG(INFO) << "PoseGraphStub created.";
  stub_ = proto::MapBuilderService::NewStub(client_channel_);
  CHECK(stub_) << "Failed to create stub.";
}

void PoseGraphStub::AddImuData(int trajectory_id,
                               const cartographer::sensor::ImuData& imu_data) {
  LOG(FATAL) << "Not implemented.";
}
void PoseGraphStub::AddOdometryData(
    int trajectory_id,
    const cartographer::sensor::OdometryData& odometry_data) {
  LOG(FATAL) << "Not implemented.";
}
void PoseGraphStub::AddFixedFramePoseData(
    int trajectory_id,
    const cartographer::sensor::FixedFramePoseData& fixed_frame_pose_data) {
  LOG(FATAL) << "Not implemented.";
}
void PoseGraphStub::FinishTrajectory(int trajectory_id) {
  LOG(FATAL) << "Not implemented.";
}
bool PoseGraphStub::IsTrajectoryFinished(int trajectory_id) {
  LOG(FATAL) << "Not implemented.";
}
void PoseGraphStub::FreezeTrajectory(int trajectory_id) {
  LOG(FATAL) << "Not implemented.";
}
void PoseGraphStub::AddSubmapFromProto(
    const cartographer::transform::Rigid3d& global_pose,
    const cartographer::mapping::proto::Submap& submap) {
  LOG(FATAL) << "Not implemented.";
}
void PoseGraphStub::AddNodeFromProto(
    const cartographer::transform::Rigid3d& global_pose,
    const cartographer::mapping::proto::Node& node) {
  LOG(FATAL) << "Not implemented.";
}
void PoseGraphStub::AddNodeToSubmap(
    const cartographer::mapping::NodeId& node_id,
    const cartographer::mapping::SubmapId& submap_id) {
  LOG(FATAL) << "Not implemented.";
}
void PoseGraphStub::AddSerializedConstraints(
    const std::vector<Constraint>& constraints) {
  LOG(FATAL) << "Not implemented.";
}
void PoseGraphStub::AddTrimmer(
    std::unique_ptr<cartographer::mapping::PoseGraphTrimmer> trimmer) {
  LOG(FATAL) << "Not implemented.";
}
void PoseGraphStub::RunFinalOptimization() { LOG(FATAL) << "Not implemented."; }
std::vector<std::vector<int>> PoseGraphStub::GetConnectedTrajectories() {
  LOG(FATAL) << "Not implemented.";
}
cartographer::mapping::PoseGraph::SubmapData PoseGraphStub::GetSubmapData(
    const cartographer::mapping::SubmapId& submap_id) {
  LOG(FATAL) << "Not implemented.";
}
cartographer::mapping::MapById<cartographer::mapping::SubmapId,
                               cartographer::mapping::PoseGraph::SubmapData>
PoseGraphStub::GetAllSubmapData() {
  return cartographer::mapping::MapById<
      cartographer::mapping::SubmapId,
      cartographer::mapping::PoseGraph::SubmapData>();
}
cartographer::transform::Rigid3d PoseGraphStub::GetLocalToGlobalTransform(
    int trajectory_id) {
  LOG(FATAL) << "Not implemented.";
}
cartographer::mapping::MapById<cartographer::mapping::NodeId,
                               cartographer::mapping::TrajectoryNode>
PoseGraphStub::GetTrajectoryNodes() {
  LOG(FATAL) << "Not implemented.";
}
cartographer::sensor::MapByTime<cartographer::sensor::ImuData>
PoseGraphStub::GetImuData() {
  LOG(FATAL) << "Not implemented.";
}
cartographer::sensor::MapByTime<cartographer::sensor::OdometryData>
PoseGraphStub::GetOdometryData() {
  LOG(FATAL) << "Not implemented.";
}
cartographer::sensor::MapByTime<cartographer::sensor::FixedFramePoseData>
PoseGraphStub::GetFixedFramePoseData() {
  LOG(FATAL) << "Not implemented.";
}
std::vector<cartographer::mapping::PoseGraph::Constraint>
PoseGraphStub::constraints() {
  LOG(FATAL) << "Not implemented.";
}
void PoseGraphStub::SetInitialTrajectoryPose(
    int from_trajectory_id, int to_trajectory_id,
    const cartographer::transform::Rigid3d& pose,
    const cartographer::common::Time time) {
  LOG(FATAL) << "Not implemented.";
}

}  // namespace mapping
}  // namespace cartographer_grpc
