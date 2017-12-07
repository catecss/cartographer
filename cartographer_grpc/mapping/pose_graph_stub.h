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

#ifndef CARTOGRAPHER_GRPC_MAPPING_POSE_GRAPH_STUB_H
#define CARTOGRAPHER_GRPC_MAPPING_POSE_GRAPH_STUB_H

#include "cartographer/mapping/pose_graph.h"
#include "cartographer_grpc/proto/map_builder_service.grpc.pb.h"
#include "cartographer_grpc/proto/map_builder_service.pb.h"
#include "grpc++/grpc++.h"

namespace cartographer_grpc {
namespace mapping {

class PoseGraphStub : public cartographer::mapping::PoseGraph {
 public:
  PoseGraphStub(std::shared_ptr<grpc::Channel> client_channel);

  void AddImuData(int trajectory_id,
                  const cartographer::sensor::ImuData& imu_data) override;
  void AddOdometryData(
      int trajectory_id,
      const cartographer::sensor::OdometryData& odometry_data) override;
  void AddFixedFramePoseData(int trajectory_id,
                             const cartographer::sensor::FixedFramePoseData&
                                 fixed_frame_pose_data) override;
  void FinishTrajectory(int trajectory_id) override;
  bool IsTrajectoryFinished(int trajectory_id) override;
  void FreezeTrajectory(int trajectory_id) override;
  void AddSubmapFromProto(
      const cartographer::transform::Rigid3d& global_pose,
      const cartographer::mapping::proto::Submap& submap) override;
  void AddNodeFromProto(
      const cartographer::transform::Rigid3d& global_pose,
      const cartographer::mapping::proto::Node& node) override;
  void AddNodeToSubmap(
      const cartographer::mapping::NodeId& node_id,
      const cartographer::mapping::SubmapId& submap_id) override;
  void AddSerializedConstraints(
      const std::vector<Constraint>& constraints) override;
  void AddTrimmer(std::unique_ptr<cartographer::mapping::PoseGraphTrimmer>
                      trimmer) override;
  void RunFinalOptimization() override;
  std::vector<std::vector<int>> GetConnectedTrajectories() override;
  SubmapData GetSubmapData(
      const cartographer::mapping::SubmapId& submap_id) override;
  cartographer::mapping::MapById<cartographer::mapping::SubmapId, SubmapData>
  GetAllSubmapData() override;
  cartographer::transform::Rigid3d GetLocalToGlobalTransform(
      int trajectory_id) override;
  cartographer::mapping::MapById<cartographer::mapping::NodeId,
                                 cartographer::mapping::TrajectoryNode>
  GetTrajectoryNodes() override;
  cartographer::sensor::MapByTime<cartographer::sensor::ImuData> GetImuData()
      override;
  cartographer::sensor::MapByTime<cartographer::sensor::OdometryData>
  GetOdometryData() override;
  cartographer::sensor::MapByTime<cartographer::sensor::FixedFramePoseData>
  GetFixedFramePoseData() override;
  std::vector<Constraint> constraints() override;
  void SetInitialTrajectoryPose(int from_trajectory_id, int to_trajectory_id,
                                const cartographer::transform::Rigid3d& pose,
                                const cartographer::common::Time time) override;

 private:
  std::shared_ptr<grpc::Channel> client_channel_;
  std::unique_ptr<proto::MapBuilderService::Stub> stub_;
};

}  // namespace mapping
}  // namespace cartographer_grpc

#endif  // CARTOGRAPHER_GRPC_MAPPING_POSE_GRAPH_STUB_H
